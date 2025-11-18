#include "display_menu.h"

#if defined(DEVICE_ROLE_HID)

#include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>
#include <RTClib.h>
#include <Wire.h>
#include <algorithm>
#include <math.h>
#include <stdlib.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "catalog.h"
#include "comm.h"
#include "config.h"
#include "input.h"
#include "motion.h"
#include "planets.h"
#include "state.h"
#include "stellarium_link.h"
#include "storage.h"
#include "text_utils.h"
#include "time_utils.h"
#include "wifi_ota.h"

namespace display_menu {

void applyOrientationState(bool known);

namespace {  

Adafruit_SSD1306 display(config::OLED_WIDTH, config::OLED_HEIGHT, &Wire, -1);
RTC_DS3231 rtc;
bool rtcAvailable = false;
SemaphoreHandle_t i2cMutex = nullptr;
StaticSemaphore_t i2cMutexBuffer;

constexpr int kLineHeight = 8;
constexpr float kTwoPi = 6.283185307f;

struct BootStar {
  float baseX;
  float baseY;
  float twinkleRate;
  float driftRate;
  float driftAmplitude;
  float phase;
};

constexpr size_t kBootStarCount = 18;
constexpr uint32_t kBootAnimationMinDurationMs = 2000;
BootStar bootStars[kBootStarCount];
bool bootAnimationActive = false;
bool bootAnimationStopRequested = false;
uint32_t bootAnimationStartMs = 0;
uint32_t bootAnimationMinEndMs = 0;

float randomFloat(float minValue, float maxValue) {
  long scale = random(0L, 10000L);
  float unit = static_cast<float>(scale) / 10000.0f;
  return minValue + (maxValue - minValue) * unit;
}

void resetBootStar(size_t index) {
  BootStar& star = bootStars[index];
  star.baseX = randomFloat(0.0f, static_cast<float>(config::OLED_WIDTH - 1));
  // Leave a little room for the status text at the bottom.
  star.baseY = randomFloat(8.0f, static_cast<float>(config::OLED_HEIGHT - 10));
  star.twinkleRate = randomFloat(0.0035f, 0.0065f);
  star.driftRate = randomFloat(0.0008f, 0.0016f) * (random(0, 2) == 0 ? -1.0f : 1.0f);
  star.driftAmplitude = randomFloat(0.5f, 2.5f);
  star.phase = randomFloat(0.0f, kTwoPi);
}

void drawStarPixel(int x, int y, int radius) {
  if (x < 0 || x >= config::OLED_WIDTH || y < 0 || y >= config::OLED_HEIGHT) {
    return;
  }
  display.drawPixel(x, y, SSD1306_WHITE);
  if (radius >= 1) {
    if (x > 0) display.drawPixel(x - 1, y, SSD1306_WHITE);
    if (x < config::OLED_WIDTH - 1) display.drawPixel(x + 1, y, SSD1306_WHITE);
    if (y > 0) display.drawPixel(x, y - 1, SSD1306_WHITE);
    if (y < config::OLED_HEIGHT - 1) display.drawPixel(x, y + 1, SSD1306_WHITE);
  }
  if (radius >= 2) {
    if (x > 0 && y > 0) display.drawPixel(x - 1, y - 1, SSD1306_WHITE);
    if (x > 0 && y < config::OLED_HEIGHT - 1)
      display.drawPixel(x - 1, y + 1, SSD1306_WHITE);
    if (x < config::OLED_WIDTH - 1 && y > 0)
      display.drawPixel(x + 1, y - 1, SSD1306_WHITE);
    if (x < config::OLED_WIDTH - 1 && y < config::OLED_HEIGHT - 1)
      display.drawPixel(x + 1, y + 1, SSD1306_WHITE);
  }
}

void drawBootAnimation(uint32_t nowMs) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("NERDSTAR");
  display.setCursor(0, 10);
  display.print("Booting");

  uint32_t elapsed = nowMs - bootAnimationStartMs;
  for (size_t i = 0; i < kBootStarCount; ++i) {
    BootStar& star = bootStars[i];
    float twinklePhase = star.phase + star.twinkleRate * static_cast<float>(elapsed);
    while (twinklePhase > kTwoPi) {
      twinklePhase -= kTwoPi;
    }
    float brightness = 0.5f + 0.5f * sinf(twinklePhase);

    float driftPhase = star.phase + star.driftRate * static_cast<float>(elapsed);
    float driftOffset = sinf(driftPhase) * star.driftAmplitude;
    int x = static_cast<int>(star.baseX + 0.5f);
    int y = static_cast<int>(star.baseY + driftOffset + 0.5f);

    if (y < 0) {
      y = 0;
    } else if (y >= config::OLED_HEIGHT) {
      y = config::OLED_HEIGHT - 1;
    }

    if (brightness < 0.2f) {
      if (brightness < 0.05f && elapsed > 750) {
        resetBootStar(i);
      }
      continue;
    }

    int radius = 0;
    if (brightness > 0.85f) {
      radius = 2;
    } else if (brightness > 0.55f) {
      radius = 1;
    }
    drawStarPixel(x, y, radius);
  }

  static const char* kSpinner[] = {"-", "\\", "|", "/"};
  int spinnerIndex = (elapsed / 300) % 4;
  display.setCursor(0, config::OLED_HEIGHT - 10);
  display.print("Waiting for link ");
  display.print(kSpinner[spinnerIndex]);
}

enum class UiState {
  StatusScreen,
  StatusDetails,
  StartupLockPrompt,
  MainMenu,
  PolarAlignMenu,
  PolarAlign,
  LockingStarMenu,
  LockingStarRefine,
  SetupMenu,
  SetRtc,
  LocationSetup,
  AxisOrientation,
  CatalogTypeBrowser,
  CatalogItemList,
  CatalogItemDetail,
  AxisCalibration,
  GotoSpeed,
  PanningSpeed,
  GotoCoordinates,
  BacklashCalibration,
};

UiState uiState = UiState::StatusScreen;

struct RtcEditState {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  DstMode dstMode;
  int fieldIndex;
  int actionIndex;
};

RtcEditState rtcEdit{2024, 1, 1, 0, 0, 0, DstMode::Auto, 0, 0};
int rtcEditScroll = 0;
constexpr int kRtcFieldCount = 8;

struct AxisCalibrationState {
  int step;
  int64_t azZero;
  int64_t azReference;
  int64_t altZero;
  int64_t altReference;
};

AxisCalibrationState axisCal{0, 0, 0, 0, 0};

struct AxisOrientationState {
  bool joystickSwapAxes;
  bool joystickInvertAz;
  bool joystickInvertAlt;
  bool motorInvertAz;
  bool motorInvertAlt;
  int fieldIndex;
  int actionIndex;
};

constexpr int kAxisOrientationFieldCount = 6;
AxisOrientationState axisOrientationState{false, false, false, false, false, 0, 0};

struct GotoProfileSteps {
  double maxSpeedAz;
  double accelerationAz;
  double decelerationAz;
  double maxSpeedAlt;
  double accelerationAlt;
  double decelerationAlt;
};

struct AxisGotoRuntime {
  int64_t finalTarget;
  int64_t compensatedTarget;
  double currentSpeed;
  int8_t desiredDirection;
  bool compensationPending;
  bool reachedFinalTarget;
};

struct GotoRuntimeState {
  bool active;
  AxisGotoRuntime az;
  AxisGotoRuntime alt;
  GotoProfileSteps profile;
  double estimatedDurationSec;
  uint32_t lastUpdateMs;
  DateTime startTime;
  double targetRaHours;
  double targetDecDegrees;
  int targetCatalogIndex;
  bool resumeTracking;
};

struct TrackingState {
  bool active;
  double targetRaHours;
  double targetDecDegrees;
  int targetCatalogIndex;
  double offsetAzDeg;
  double offsetAltDeg;
  bool userAdjusting;
};

GotoRuntimeState gotoRuntime{false,
                             {0, 0, 0.0, 0, false, false},
                             {0, 0, 0.0, 0, false, false},
                             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                             0.0,
                             0,
                             DateTime(),
                             0.0,
                             0.0,
                             -1,
                             true};

TrackingState tracking{false, 0.0, 0.0, -1, 0.0, 0.0, false};

enum class SpeedEditMode { Goto, Panning };

struct SpeedProfileState {
  float maxSpeed;
  float acceleration;
  float deceleration;
  int fieldIndex;
  SpeedEditMode mode;
  int actionIndex;
};

struct BacklashCalibrationState {
  int step;
  int64_t azStart;
  int64_t azEnd;
  int64_t altStart;
  int64_t altEnd;
};

SpeedProfileState speedProfileState{0.0f, 0.0f, 0.0f, 0, SpeedEditMode::Goto, 0};
BacklashCalibrationState backlashState{0, 0, 0, 0, 0};

struct LocationEditState {
  double latitudeDeg;
  double longitudeDeg;
  int32_t timezoneMinutes;
  int fieldIndex;
  int actionIndex;
};

struct GotoCoordinateState {
  int raHours;
  int raMinutes;
  int raSeconds;
  bool decNegative;
  int decDegrees;
  int decArcMinutes;
  int decArcSeconds;
  int fieldIndex;
};

constexpr int kGotoCoordinateFieldCount = 8;

LocationEditState locationEdit{0.0, 0.0, 60, 0, 0};
GotoCoordinateState gotoCoordinateState{0, 0, 0, false, 0, 0, 0, 0};
double manualGotoRaHours = 0.0;
double manualGotoDecDegrees = 0.0;
constexpr int kLocationFieldCount = 4;

constexpr int kSpeedProfileFieldCount = 4;

String selectedObjectName;
String gotoTargetName;

int mainMenuIndex = 0;
int mainMenuScroll = 0;
constexpr const char* kMainMenuItems[] = {"Status",
                                          "Polar Align",
                                          "Start Tracking",
                                          "Stop Tracking",
                                          "Catalog",
                                          "Goto Selected",
                                          "Goto RA/Dec",
                                          "Park",
                                          "Setup"};
constexpr size_t kMainMenuCount = sizeof(kMainMenuItems) / sizeof(kMainMenuItems[0]);

int polarAlignMenuIndex = 0;
int polarAlignMenuScroll = 0;
constexpr const char* kPolarAlignMenuItems[] = {"Lock Polaris",
                                                "Refine Alignment",
                                                "Clear Refinements",
                                                "Back"};
constexpr size_t kPolarAlignMenuCount =
    sizeof(kPolarAlignMenuItems) / sizeof(kPolarAlignMenuItems[0]);
constexpr int kPolarAlignMenuLockIndex = 0;
constexpr int kPolarAlignMenuRefineIndex = 1;
constexpr int kPolarAlignMenuClearIndex = 2;
constexpr int kPolarAlignMenuBackIndex = 3;

int setupMenuIndex = 0;
int setupMenuScroll = 0;
constexpr const char* kSetupMenuItems[] = {
    "Set RTC",      "Set Location", "Cal Joystick", "Axis Setup",
    "Cal Axes",     "Goto Speed",   "Pan Speed",    "Cal Backlash",
    "WiFi OTA",     "WiFi AP",      "Stellarium",   "Back"};
constexpr size_t kSetupMenuCount = sizeof(kSetupMenuItems) / sizeof(kSetupMenuItems[0]);

constexpr int kSetupMenuRtcIndex = 0;
constexpr int kSetupMenuLocationIndex = 1;
constexpr int kSetupMenuJoystickIndex = 2;
constexpr int kSetupMenuAxisOrientationIndex = 3;
constexpr int kSetupMenuAxisCalIndex = 4;
constexpr int kSetupMenuGotoSpeedIndex = 5;
constexpr int kSetupMenuPanSpeedIndex = 6;
constexpr int kSetupMenuBacklashIndex = 7;
constexpr int kSetupMenuWifiIndex = 8;
constexpr int kSetupMenuWifiApIndex = 9;
constexpr int kSetupMenuStellariumIndex = 10;
constexpr int kSetupMenuBackIndex = 11;

int catalogTypeIndex = 0;
int catalogTypeObjectIndex = 0;
int catalogIndex = 0;
int catalogItemScroll = 0;
int catalogDetailMenuIndex = 0;
bool catalogDetailSelectingAction = false;
constexpr int kCatalogDetailMenuCount = 2;

constexpr const char* kLockingStarCandidates[] = {"Dubhe",     "Alioth",   "Arcturus",
                                                  "Vega",      "Altair",   "Deneb",
                                                  "Capella",   "Betelgeuse", "Rigel",
                                                  "Aldebaran", "Spica",    "Regulus",
                                                  "Procyon",   "Sirius",   "Fomalhaut"};
constexpr size_t kLockingStarCandidateCount =
    sizeof(kLockingStarCandidates) / sizeof(kLockingStarCandidates[0]);

struct LockingStarOption {
  String name;
  int catalogIndex;
  double raHours;
  double decDegrees;
  double azDeg;
  double altDeg;
};

constexpr size_t kMaxLockingStarOptions = kLockingStarCandidateCount;
LockingStarOption lockingStarOptions[kMaxLockingStarOptions];
size_t lockingStarOptionCount = 0;
int lockingStarSelectionIndex = 0;
int lockingStarScroll = 0;
bool lockingStarFlowActive = false;
bool lockingStarGotoInProgress = false;
bool lockingStarPendingRefine = false;
double lockingStarPendingRaHours = 0.0;
double lockingStarPendingDecDegrees = 0.0;
int lockingStarPendingCatalogIndex = -1;
String lockingStarPendingName;
bool lockingStarReturnToPolarMenu = false;

String infoMessage;
uint32_t infoUntil = 0;
portMUX_TYPE displayMux = portMUX_INITIALIZER_UNLOCKED;
bool orientationKnown = false;

struct StellariumStatusView {
  bool connected;
  double raHours;
  double decDegrees;
};

StellariumStatusView stellariumStatus{false, 0.0, 0.0};

struct OrientationModel {
  OrientationModel();

  void loadFromConfig(const SystemConfig& config);
  void reset();
  void addSample(double expectedAz, double expectedAlt, double measuredAz, double measuredAlt);
  double toSkyAz(double physicalAz) const;
  double toSkyAlt(double physicalAlt) const;
  double toPhysicalAz(double skyAz) const;
  double toPhysicalAlt(double skyAlt) const;
  bool hasCalibration() const;
  double azBias() const;
  double altBias() const;
  double totalWeight() const;

 private:
  double azBiasDeg_;
  double altBiasDeg_;
  double totalWeight_;
  double sinAccumulator_;
  double cosAccumulator_;
  double altAccumulator_;
};

OrientationModel orientationModel;
bool startupPromptActive = false;
int startupPromptIndex = 0;
constexpr const char* kStartupPromptItems[] = {
    "Use saved lock", "New Polaris lock", "Discard lock"};
constexpr size_t kStartupPromptCount = sizeof(kStartupPromptItems) / sizeof(kStartupPromptItems[0]);

class MutexLock {
 public:
  explicit MutexLock(SemaphoreHandle_t handle) : handle_(handle), locked_(false) {
    if (!handle_) {
      locked_ = true;
      return;
    }
    locked_ = xSemaphoreTakeRecursive(handle_, portMAX_DELAY) == pdTRUE;
  }

  ~MutexLock() {
    if (locked_ && handle_) {
      xSemaphoreGiveRecursive(handle_);
    }
  }

  bool locked() const { return locked_; }

 private:
  SemaphoreHandle_t handle_;
  bool locked_;
};

void setUiState(UiState state) {
  uiState = state;
}

void abortGoto();
bool raDecToAltAz(const DateTime& when,
                  double raHours,
                  double decDegrees,
                  double& azDeg,
                  double& altDeg);
bool startGotoToObject(const CatalogObject& object, int catalogIndex);
bool startGotoToCoordinates(double raHours, double decDegrees, const String& label);
bool startTrackingCurrentOrientation();
bool startParkPosition();
void drawStartupLockPrompt();
void handleStartupLockPromptInput(int delta);
void drawLockingStarMenu();
void drawLockingStarRefine();
void handleLockingStarMenuInput(int delta);
void handleLockingStarRefineInput();
void drawPolarAlignMenu();
void handlePolarAlignMenuInput(int delta);
void enterPolarAlignMenu();

void drawHeader() {
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("NERDSTAR");
  if (rtcAvailable) {
    time_t utcEpoch = rtc.now().unixtime();
    DateTime now = time_utils::applyTimezone(utcEpoch);
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buffer, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(config::OLED_WIDTH - w, 0);
    display.print(buffer);
  }
}

int lineY(int top, int row) { return top + row * kLineHeight; }

int computeVisibleRows(int topMargin, int bottomMargin) {
  int available = config::OLED_HEIGHT - topMargin - bottomMargin;
  if (available <= 0) {
    return 0;
  }
  int rows = available / kLineHeight;
  if (rows * kLineHeight < available) {
    rows += 1;
  }
  return rows;
}

void ensureSelectionVisible(int& scroll, int selected, int visibleRows, size_t total) {
  if (visibleRows <= 0) {
    scroll = 0;
    return;
  }
  int maxScroll = std::max<int>(0, static_cast<int>(total) - visibleRows);
  if (scroll > maxScroll) {
    scroll = maxScroll;
  }
  if (selected < scroll) {
    scroll = selected;
  } else if (selected >= scroll + visibleRows) {
    scroll = selected - visibleRows + 1;
  }
  if (scroll < 0) {
    scroll = 0;
  }
}

void formatRa(double hours, char* buffer, size_t length) {
  double normalized = fmod(hours, 24.0);
  if (normalized < 0.0) normalized += 24.0;
  int h = static_cast<int>(normalized);
  double minutesFloat = (normalized - h) * 60.0;
  int m = static_cast<int>(minutesFloat);
  int s = static_cast<int>((minutesFloat - m) * 60.0 + 0.5);
  if (s >= 60) {
    s -= 60;
    m += 1;
  }
  if (m >= 60) {
    m -= 60;
    h = (h + 1) % 24;
  }
  snprintf(buffer, length, "%02dh %02dm %02ds", h, m, s);
}

void formatDec(double degrees, char* buffer, size_t length) {
  char sign = degrees >= 0.0 ? '+' : '-';
  double absVal = fabs(degrees);
  int d = static_cast<int>(absVal);
  double minutesFloat = (absVal - d) * 60.0;
  int m = static_cast<int>(minutesFloat);
  int s = static_cast<int>((minutesFloat - m) * 60.0 + 0.5);
  if (s >= 60) {
    s -= 60;
    m += 1;
  }
  if (m >= 60) {
    m -= 60;
    d += 1;
  }
  snprintf(buffer, length, "%c%02d%c %02d' %02d\"", sign, d, kDegreeSymbol, m, s);
}

String makeRaDecLabel(double raHours, double decDegrees) {
  double normalizedRa = fmod(raHours, 24.0);
  if (normalizedRa < 0.0) normalizedRa += 24.0;
  int h = static_cast<int>(floor(normalizedRa));
  double raMinutesFloat = (normalizedRa - h) * 60.0;
  int m = static_cast<int>(floor(raMinutesFloat));
  double raSecondsFloat = (raMinutesFloat - m) * 60.0;
  int s = static_cast<int>(llround(raSecondsFloat));
  if (s >= 60) {
    s -= 60;
    m += 1;
  }
  if (m >= 60) {
    m -= 60;
    h = (h + 1) % 24;
  }
  char raBuffer[16];
  snprintf(raBuffer, sizeof(raBuffer), "%02d:%02d:%02d", h, m, s);

  char sign = decDegrees >= 0.0 ? '+' : '-';
  double absDec = fabs(decDegrees);
  if (absDec > 90.0) absDec = 90.0;
  int d = static_cast<int>(floor(absDec));
  double decMinutesFloat = (absDec - d) * 60.0;
  int decMinutes = static_cast<int>(floor(decMinutesFloat));
  double decSecondsFloat = (decMinutesFloat - decMinutes) * 60.0;
  int decSeconds = static_cast<int>(llround(decSecondsFloat));
  if (decSeconds >= 60) {
    decSeconds -= 60;
    decMinutes += 1;
  }
  if (decMinutes >= 60) {
    decMinutes -= 60;
    if (d < 90) d += 1;
  }
  if (d >= 90) {
    d = 90;
    decMinutes = 0;
    decSeconds = 0;
  }
  char decBuffer[16];
  snprintf(decBuffer, sizeof(decBuffer), "%c%02d%c%02d'%02d\"", sign, d, kDegreeSymbol, decMinutes, decSeconds);

  String label(raBuffer);
  label += " / ";
  label += decBuffer;
  return label;
}

bool fetchInfoMessage(String& message) {
  bool active = false;
  portENTER_CRITICAL(&displayMux);
  if (!infoMessage.isEmpty()) {
    uint32_t now = millis();
    if (now <= infoUntil) {
      message = infoMessage;
      active = true;
    } else {
      infoMessage = "";
    }
  }
  portEXIT_CRITICAL(&displayMux);
  return active;
}

double degToRad(double degrees) { return degrees * DEG_TO_RAD; }

double radToDeg(double radians) { return radians * RAD_TO_DEG; }

double wrapAngle360(double degrees) {
  double wrapped = fmod(degrees, 360.0);
  if (wrapped < 0.0) wrapped += 360.0;
  return wrapped;
}

double wrapAngle180(double degrees) {
  double wrapped = fmod(degrees + 180.0, 360.0);
  if (wrapped < 0.0) wrapped += 360.0;
  return wrapped - 180.0;
}

double shortestAngularDistance(double from, double to) {
  double diff = wrapAngle180(to - from);
  return diff;
}

double computeOrientationSampleWeight(double azErrorDeg, double altErrorDeg) {
  double magnitude = sqrt(azErrorDeg * azErrorDeg + altErrorDeg * altErrorDeg);
  double weight = 1.0 / (1.0 + magnitude / 3.0);
  if (weight < 0.1) weight = 0.1;
  if (weight > 1.0) weight = 1.0;
  return weight;
}

OrientationModel::OrientationModel()
    : azBiasDeg_(0.0),
      altBiasDeg_(0.0),
      totalWeight_(0.0),
      sinAccumulator_(0.0),
      cosAccumulator_(0.0),
      altAccumulator_(0.0) {}

void OrientationModel::loadFromConfig(const SystemConfig& config) {
  azBiasDeg_ = config.orientationAzBiasDeg;
  altBiasDeg_ = config.orientationAltBiasDeg;
  totalWeight_ = std::max(0.0, static_cast<double>(config.orientationSampleWeight));
  if (totalWeight_ > 0.0) {
    double azRad = degToRad(azBiasDeg_);
    sinAccumulator_ = sin(azRad) * totalWeight_;
    cosAccumulator_ = cos(azRad) * totalWeight_;
    altAccumulator_ = altBiasDeg_ * totalWeight_;
  } else {
    sinAccumulator_ = 0.0;
    cosAccumulator_ = 0.0;
    altAccumulator_ = 0.0;
    azBiasDeg_ = 0.0;
    altBiasDeg_ = 0.0;
  }
}

void OrientationModel::reset() {
  azBiasDeg_ = 0.0;
  altBiasDeg_ = 0.0;
  totalWeight_ = 0.0;
  sinAccumulator_ = 0.0;
  cosAccumulator_ = 0.0;
  altAccumulator_ = 0.0;
}

void OrientationModel::addSample(double expectedAz,
                                 double expectedAlt,
                                 double measuredAz,
                                 double measuredAlt) {
  double azError = shortestAngularDistance(expectedAz, measuredAz);
  double altError = measuredAlt - expectedAlt;
  double weight = computeOrientationSampleWeight(azError, altError);
  if (weight <= 0.0) {
    return;
  }
  constexpr double kDecay = 0.9;
  sinAccumulator_ *= kDecay;
  cosAccumulator_ *= kDecay;
  altAccumulator_ *= kDecay;
  totalWeight_ *= kDecay;

  double azRad = degToRad(azError);
  sinAccumulator_ += sin(azRad) * weight;
  cosAccumulator_ += cos(azRad) * weight;
  altAccumulator_ += altError * weight;
  totalWeight_ += weight;

  if (totalWeight_ < 1e-6) {
    reset();
    storage::clearOrientationModel();
    return;
  }

  double magnitude = sqrt(sinAccumulator_ * sinAccumulator_ + cosAccumulator_ * cosAccumulator_);
  if (magnitude < 1e-6) {
    azBiasDeg_ = 0.0;
    sinAccumulator_ = 0.0;
    cosAccumulator_ = totalWeight_;
  } else {
    azBiasDeg_ = radToDeg(atan2(sinAccumulator_, cosAccumulator_));
  }
  altBiasDeg_ = altAccumulator_ / totalWeight_;
  storage::setOrientationModel(azBiasDeg_, altBiasDeg_, totalWeight_);
}

double OrientationModel::toSkyAz(double physicalAz) const {
  return wrapAngle360(physicalAz - azBiasDeg_);
}

double OrientationModel::toSkyAlt(double physicalAlt) const { return physicalAlt - altBiasDeg_; }

double OrientationModel::toPhysicalAz(double skyAz) const {
  return wrapAngle360(skyAz + azBiasDeg_);
}

double OrientationModel::toPhysicalAlt(double skyAlt) const { return skyAlt + altBiasDeg_; }

bool OrientationModel::hasCalibration() const { return totalWeight_ > 0.0; }

double OrientationModel::azBias() const { return azBiasDeg_; }

double OrientationModel::altBias() const { return altBiasDeg_; }

double OrientationModel::totalWeight() const { return totalWeight_; }

double applyAtmosphericRefraction(double geometricAltitudeDeg) {
  if (geometricAltitudeDeg < -1.0 || geometricAltitudeDeg > 90.0) {
    return geometricAltitudeDeg;
  }

  double altitudeWithOffset =
      geometricAltitudeDeg + 10.3 / (geometricAltitudeDeg + 5.11);
  double refractionArcMinutes =
      1.02 / tan(degToRad(altitudeWithOffset));
  return geometricAltitudeDeg + refractionArcMinutes / 60.0;
}

DateTime currentDateTime() {
  const SystemConfig& config = storage::getConfig();
  if (rtcAvailable) {
    MutexLock lock(i2cMutex);
    if (lock.locked()) {
      time_t utcEpoch = rtc.now().unixtime();
      return time_utils::applyTimezone(utcEpoch);
    }
  }
  if (config.lastRtcEpoch != 0) {
    return time_utils::applyTimezone(static_cast<time_t>(config.lastRtcEpoch));
  }
  return DateTime(2024, 1, 1, 0, 0, 0);
}

DateTime toUtc(const DateTime& local) {
  time_t epoch = time_utils::toUtcEpoch(local);
  return DateTime(epoch);
}

double hourFraction(const DateTime& time) {
  return time.hour() + time.minute() / 60.0 + time.second() / 3600.0;
}

double localSiderealDegrees(const DateTime& time) {
  DateTime utc = toUtc(time);
  double jd = planets::julianDay(utc.year(), utc.month(), utc.day(),
                                 static_cast<float>(hourFraction(utc)));
  double T = (jd - 2451545.0) / 36525.0;
  double lst = 280.46061837 + 360.98564736629 * (jd - 2451545.0) +
               0.000387933 * T * T - (T * T * T) / 38710000.0 +
               storage::getConfig().observerLongitudeDeg;
  return wrapAngle360(lst);
}

int findCatalogIndexForObject(const CatalogObject* target) {
  if (!target) {
    return -1;
  }
  size_t total = catalog::size();
  for (size_t i = 0; i < total; ++i) {
    const CatalogObject* candidate = catalog::get(i);
    if (candidate == target) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void resetLockingStarFlow() {
  lockingStarFlowActive = false;
  lockingStarGotoInProgress = false;
  lockingStarPendingRefine = false;
  lockingStarPendingRaHours = 0.0;
  lockingStarPendingDecDegrees = 0.0;
  lockingStarPendingCatalogIndex = -1;
  lockingStarPendingName = "";
  lockingStarOptionCount = 0;
  lockingStarSelectionIndex = 0;
  lockingStarScroll = 0;
  lockingStarReturnToPolarMenu = false;
}

void populateLockingStarOptions() {
  lockingStarOptionCount = 0;
  DateTime now = currentDateTime();
  for (size_t i = 0; i < kLockingStarCandidateCount; ++i) {
    if (lockingStarOptionCount >= kMaxLockingStarOptions) {
      break;
    }
    const char* candidateName = kLockingStarCandidates[i];
    const CatalogObject* object = catalog::findByName(String(candidateName));
    if (!object) {
      continue;
    }
    int catalogIndex = findCatalogIndexForObject(object);
    if (catalogIndex < 0) {
      continue;
    }
    double azDeg = 0.0;
    double altDeg = 0.0;
    if (!raDecToAltAz(now, object->raHours, object->decDegrees, azDeg, altDeg)) {
      continue;
    }
    if (altDeg < motion::getMinAltitudeDegrees()) {
      continue;
    }
    LockingStarOption& option = lockingStarOptions[lockingStarOptionCount++];
    option.name = sanitizeForDisplay(object->name);
    option.catalogIndex = catalogIndex;
    option.raHours = object->raHours;
    option.decDegrees = object->decDegrees;
    option.azDeg = azDeg;
    option.altDeg = altDeg;
  }
  lockingStarSelectionIndex = 0;
  lockingStarScroll = 0;
}

void showLockingStarMenu() {
  populateLockingStarOptions();
  setUiState(UiState::LockingStarMenu);
  systemState.menuMode = lockingStarReturnToPolarMenu ? MenuMode::PolarAlign : MenuMode::Status;
}

bool computeLockingStarErrors(double& azErrorDeg, double& altErrorDeg) {
  if (!lockingStarPendingRefine) {
    return false;
  }
  DateTime now = currentDateTime();
  double expectedAz = 0.0;
  double expectedAlt = 0.0;
  if (!raDecToAltAz(now, lockingStarPendingRaHours, lockingStarPendingDecDegrees, expectedAz,
                    expectedAlt)) {
    return false;
  }
  double physicalAz = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
  double physicalAlt = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));
  double currentAz = orientationModel.toSkyAz(physicalAz);
  double currentAlt = orientationModel.toSkyAlt(physicalAlt);
  azErrorDeg = shortestAngularDistance(expectedAz, currentAz);
  altErrorDeg = currentAlt - expectedAlt;
  return true;
}

void startLockingStarFollowup(bool returnToMenu = false) {
  if (catalog::size() == 0) {
    if (returnToMenu) {
      showInfo("Catalog missing");
    }
    return;
  }
  lockingStarFlowActive = true;
  lockingStarGotoInProgress = false;
  lockingStarPendingRefine = false;
  lockingStarPendingCatalogIndex = -1;
  lockingStarPendingName = "";
  lockingStarReturnToPolarMenu = returnToMenu;
  showLockingStarMenu();
}

bool startLockingStarGoto(const LockingStarOption& option) {
  if (option.catalogIndex < 0 || option.catalogIndex >= static_cast<int>(catalog::size())) {
    showInfo("Star unavailable");
    return false;
  }
  const CatalogObject* object = catalog::get(static_cast<size_t>(option.catalogIndex));
  if (!object) {
    showInfo("Star unavailable");
    return false;
  }
  if (!startGotoToObject(*object, option.catalogIndex)) {
    return false;
  }
  selectedObjectName = sanitizeForDisplay(object->name);
  gotoTargetName = sanitizeForDisplay(object->name);
  systemState.selectedCatalogIndex = option.catalogIndex;
  lockingStarGotoInProgress = true;
  lockingStarPendingRefine = false;
  lockingStarPendingCatalogIndex = option.catalogIndex;
  lockingStarPendingRaHours = option.raHours;
  lockingStarPendingDecDegrees = option.decDegrees;
  lockingStarPendingName = sanitizeForDisplay(object->name);
  setUiState(UiState::StatusScreen);
  systemState.menuMode = MenuMode::Status;
  return true;
}

void finishLockingStarFlow() {
  bool returnToMenu = lockingStarReturnToPolarMenu;
  resetLockingStarFlow();
  lockingStarReturnToPolarMenu = false;
  if (returnToMenu) {
    systemState.menuMode = MenuMode::PolarAlign;
    setUiState(UiState::PolarAlignMenu);
  } else {
    systemState.menuMode = MenuMode::Status;
    setUiState(UiState::StatusScreen);
  }
}

void finalizeLockingStarRefinement() {
  if (!lockingStarPendingRefine) {
    finishLockingStarFlow();
    return;
  }
  DateTime now = currentDateTime();
  double expectedAz = 0.0;
  double expectedAlt = 0.0;
  if (!raDecToAltAz(now, lockingStarPendingRaHours, lockingStarPendingDecDegrees, expectedAz,
                    expectedAlt)) {
    showInfo("Star unavailable");
    return;
  }
  double currentAz = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
  double currentAlt = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));
  double azError = shortestAngularDistance(expectedAz, currentAz);
  double altError = currentAlt - expectedAlt;

  orientationModel.addSample(expectedAz, expectedAlt, currentAz, currentAlt);

  stopTracking();
  motion::setStepCount(Axis::Az, motion::azDegreesToSteps(expectedAz));
  motion::setStepCount(Axis::Alt, motion::altDegreesToSteps(expectedAlt));
  systemState.polarAligned = true;
  storage::setPolarAligned(true);
  applyOrientationState(true);
  bool trackingStarted = startTrackingCurrentOrientation();

  char message[64];
  snprintf(message, sizeof(message), "Lock saved dAz=%+.2f%c dAlt=%+.2f%c", azError, kDegreeSymbol,
           altError, kDegreeSymbol);
  String infoMessage(message);
  if (orientationModel.hasCalibration()) {
    char modelBuffer[48];
    snprintf(modelBuffer, sizeof(modelBuffer), " mdl=%+.2f%c/%+.2f%c", orientationModel.azBias(),
             kDegreeSymbol, orientationModel.altBias(), kDegreeSymbol);
    infoMessage += modelBuffer;
  }
  if (!trackingStarted) {
    infoMessage += " (tracking off)";
  }
  showInfo(infoMessage, 4000);

  finishLockingStarFlow();
}

void getObjectRaDecAt(const CatalogObject& object,
                      const DateTime& when,
                      double secondsAhead,
                      double& raHours,
                      double& decDegrees,
                      DateTime* futureTime) {
  DateTime future = when + TimeSpan(0, 0, 0, static_cast<int32_t>(secondsAhead));
  double fractional = secondsAhead - floor(secondsAhead);
  raHours = object.raHours;
  decDegrees = object.decDegrees;

  if (futureTime) {
    *futureTime = future;
  }

  PlanetId planetId;
  if (object.type.equalsIgnoreCase("planet") &&
      planets::planetFromString(object.name, planetId)) {
    DateTime futureUtc = toUtc(future);
    float jd = planets::julianDay(
        futureUtc.year(), futureUtc.month(), futureUtc.day(),
        static_cast<float>(hourFraction(futureUtc) + fractional / 3600.0));
    PlanetPosition position;
    if (planets::computePlanet(planetId, jd, position)) {
      raHours = position.raHours;
      decDegrees = position.decDegrees;
    }
  }
}

bool raDecToAltAz(const DateTime& when,
                  double raHours,
                  double decDegrees,
                  double& azimuthDeg,
                  double& altitudeDeg) {
  double lstDeg = localSiderealDegrees(when);
  double raDeg = raHours * 15.0;
  double haDeg = wrapAngle180(lstDeg - raDeg);
  double latRad = degToRad(storage::getConfig().observerLatitudeDeg);
  double haRad = degToRad(haDeg);
  double decRad = degToRad(decDegrees);

  double sinAlt = sin(decRad) * sin(latRad) + cos(decRad) * cos(latRad) * cos(haRad);
  sinAlt = std::clamp(sinAlt, -1.0, 1.0);
  double geometricAltitudeDeg = radToDeg(asin(sinAlt));

  double cosAz = (sin(decRad) - sinAlt * sin(latRad)) /
                 (cos(degToRad(geometricAltitudeDeg)) * cos(latRad));
  cosAz = std::clamp(cosAz, -1.0, 1.0);
  double azRad = acos(cosAz);
  if (sin(haRad) > 0) {
    azRad = 2 * PI - azRad;
  }
  azimuthDeg = wrapAngle360(radToDeg(azRad));
  altitudeDeg = applyAtmosphericRefraction(geometricAltitudeDeg);
  return altitudeDeg > -5.0;  // allow slight tolerance below horizon
}

bool altAzToRaDec(const DateTime& when,
                  double azimuthDeg,
                  double altitudeDeg,
                  double& raHours,
                  double& decDegrees) {
  double lstDeg = localSiderealDegrees(when);
  double latRad = degToRad(storage::getConfig().observerLatitudeDeg);
  double altRad = degToRad(altitudeDeg);
  double azRad = degToRad(azimuthDeg);

  double sinDec = sin(altRad) * sin(latRad) + cos(altRad) * cos(latRad) * cos(azRad);
  sinDec = std::clamp(sinDec, -1.0, 1.0);
  double decRad = asin(sinDec);

  double sinHa = -sin(azRad) * cos(altRad);
  double cosHa = cos(altRad) * sin(latRad) - sin(altRad) * cos(latRad) * cos(azRad);
  double haRad = atan2(sinHa, cosHa);
  double haDeg = wrapAngle180(radToDeg(haRad));

  double raDeg = wrapAngle360(lstDeg - haDeg);
  raHours = raDeg / 15.0;
  decDegrees = radToDeg(decRad);
  return true;
}

bool computeCurrentEquatorial(double& raHours, double& decDegrees) {
  if (!orientationKnown) {
    return false;
  }
  double physicalAz = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
  double physicalAlt = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));
  double skyAz = orientationModel.toSkyAz(physicalAz);
  double skyAlt = orientationModel.toSkyAlt(physicalAlt);
  DateTime now = currentDateTime();
  return altAzToRaDec(now, skyAz, skyAlt, raHours, decDegrees);
}

void setStellariumStatus(bool connected, double raHours, double decDegrees) {
  stellariumStatus.connected = connected;
  if (connected) {
    stellariumStatus.raHours = raHours;
    stellariumStatus.decDegrees = decDegrees;
  }
}

GotoProfileSteps toProfileSteps(const GotoProfile& profile, const AxisCalibration& cal) {
  GotoProfileSteps result{};
  result.maxSpeedAz = profile.maxSpeedDegPerSec * cal.stepsPerDegreeAz;
  result.accelerationAz = profile.accelerationDegPerSec2 * cal.stepsPerDegreeAz;
  result.decelerationAz = profile.decelerationDegPerSec2 * cal.stepsPerDegreeAz;
  result.maxSpeedAlt = profile.maxSpeedDegPerSec * cal.stepsPerDegreeAlt;
  result.accelerationAlt = profile.accelerationDegPerSec2 * cal.stepsPerDegreeAlt;
  result.decelerationAlt = profile.decelerationDegPerSec2 * cal.stepsPerDegreeAlt;
  return result;
}

double computeTravelTimeSteps(double distanceSteps,
                              double maxSpeed,
                              double accel,
                              double decel) {
  double distance = fabs(distanceSteps);
  if (distance < 1.0) {
    return 0.0;
  }
  maxSpeed = std::max(maxSpeed, 1.0);
  accel = std::max(accel, 1.0);
  decel = std::max(decel, 1.0);
  double distAccel = (maxSpeed * maxSpeed) / (2.0 * accel);
  double distDecel = (maxSpeed * maxSpeed) / (2.0 * decel);
  if (distance >= distAccel + distDecel) {
    double cruise = distance - distAccel - distDecel;
    return maxSpeed / accel + maxSpeed / decel + cruise / maxSpeed;
  }
  double peakSpeed = sqrt((2.0 * distance * accel * decel) / (accel + decel));
  return peakSpeed / accel + peakSpeed / decel;
}

void drawStatus(bool diagnostics) {
  double azDeg = 0.0;
  double altDeg = 0.0;
  if (orientationKnown) {
    azDeg = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
    altDeg = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));
  }
  char azBuffer[24];
  char altBuffer[24];
  snprintf(azBuffer, sizeof(azBuffer), "%06.2f%c", azDeg, kDegreeSymbol);
  formatDec(altDeg, altBuffer, sizeof(altBuffer));

  constexpr int kStatusTop = 10;
  int row = 0;
  int maxRows = ((config::OLED_HEIGHT - kStatusTop) / kLineHeight) + 1;
  auto nextY = [&]() -> int {
    if (row >= maxRows) {
      return -1;
    }
    int y = lineY(kStatusTop, row);
    ++row;
    return y;
  };

  if (int y = nextY(); y >= 0) {
    display.setCursor(0, y);
    display.print("Az: ");
    display.print(azBuffer);
  }
  if (int y = nextY(); y >= 0) {
    display.setCursor(0, y);
    display.print("Alt: ");
    display.print(altBuffer);
  }
  if (int y = nextY(); y >= 0) {
    display.setCursor(0, y);
    display.print("Align: ");
    display.print(systemState.polarAligned ? "Yes" : "No");
    display.print("  Trk: ");
    display.print(systemState.trackingActive ? "On" : "Off");
  }

  if (stellariumStatus.connected) {
    if (int y = nextY(); y >= 0) {
      display.setCursor(0, y);
      display.print("Stellarium: On");
    }
    if (int y = nextY(); y >= 0) {
      char raBuffer[24];
      char decBuffer[24];
      formatRa(stellariumStatus.raHours, raBuffer, sizeof(raBuffer));
      formatDec(stellariumStatus.decDegrees, decBuffer, sizeof(decBuffer));
      display.setCursor(0, y);
      display.print("IST: ");
      display.print(raBuffer);
      display.print(" ");
      display.print(decBuffer);
    }
  }

  if (diagnostics) {
    auto printDetail = [&](const char* label, const String& value) {
      if (int y = nextY(); y >= 0) {
        display.setCursor(0, y);
        display.print(label);
        display.print(value);
      }
    };
    if (systemState.gotoActive) {
      printDetail("Goto: ", gotoTargetName);
    } else if (tracking.active) {
      printDetail("Track: ", gotoTargetName);
    } else if (!selectedObjectName.isEmpty()) {
      printDetail("Sel: ", selectedObjectName);
    }
    if (int y = nextY(); y >= 0) {
      display.setCursor(0, y);
      display.printf("Joy:%+0.2f,%+0.2f Btn:%s", systemState.joystickX, systemState.joystickY,
                     systemState.joystickButtonPressed ? "On" : "Off");
    }
    if (int y = nextY(); y >= 0) {
      display.setCursor(0, y);
      display.print("Link: ");
      display.print(systemState.mountLinkReady ? "Ready" : "Offline");
      display.print(" Cmd: ");
      display.print(systemState.manualCommandOk ? "OK" : "Err");
    }
    if (int y = nextY(); y >= 0) {
      display.setCursor(0, y);
      display.print("Joy=Close Enc=Menu");
    }
  } else {
    if (!selectedObjectName.isEmpty()) {
      if (int y = nextY(); y >= 0) {
        display.setCursor(0, y);
        display.print("Sel: ");
        display.print(selectedObjectName);
      }
    }
    if (systemState.gotoActive) {
      if (int y = nextY(); y >= 0) {
        display.setCursor(0, y);
        display.print("Goto: ");
        display.print(gotoTargetName);
      }
    } else if (tracking.active) {
      if (int y = nextY(); y >= 0) {
        display.setCursor(0, y);
        display.print("Track: ");
        display.print(gotoTargetName);
      }
    }
  }
  if (uiState == UiState::StatusScreen) {
    display.setCursor(0, 50);
    display.printf("Joy:%+0.2f,%+0.2f", systemState.joystickX, systemState.joystickY);
    display.setCursor(90, 50);
    display.print(systemState.joystickButtonPressed ? "BTN" : "---");
    display.setCursor(112, 50);
    display.print(systemState.mountLinkReady ? "L" : "!");
    display.setCursor(118, 50);
    display.print(systemState.manualCommandOk ? "C" : "!");
  }
}

void drawStartupLockPrompt() {
  display.setCursor(0, 12);
  display.print("Saved lock found");
  display.setCursor(0, 20);
  display.print("Select action:");

  int y = 32;
  for (size_t i = 0; i < kStartupPromptCount; ++i) {
    display.setCursor(0, y);
    display.print((startupPromptIndex == static_cast<int>(i)) ? "> " : "  ");
    display.print(kStartupPromptItems[i]);
    y += kLineHeight;
  }

  display.setCursor(0, 32 + static_cast<int>(kStartupPromptCount) * kLineHeight);
  display.print("Enc/Joy=Confirm");
}

void drawLockingStarMenu() {
  display.setCursor(0, 12);
  display.print("Refine alignment");
  int listTop = 24;
  int visibleRows = computeVisibleRows(listTop, kLineHeight);
  if (lockingStarOptionCount == 0) {
    display.setCursor(0, listTop);
    display.print("No bright stars");
    display.setCursor(0, listTop + kLineHeight);
    display.print("Enc/Joy=Skip");
    return;
  }

  if (visibleRows <= 0) {
    visibleRows = 1;
  }

  if (lockingStarSelectionIndex < lockingStarScroll) {
    lockingStarScroll = lockingStarSelectionIndex;
  }
  if (lockingStarSelectionIndex >= lockingStarScroll + visibleRows) {
    lockingStarScroll = lockingStarSelectionIndex - visibleRows + 1;
  }

  for (int row = 0; row < visibleRows; ++row) {
    int optionIndex = lockingStarScroll + row;
    if (optionIndex >= static_cast<int>(lockingStarOptionCount)) {
      break;
    }
    int y = lineY(listTop, row);
    display.setCursor(0, y);
    display.print((optionIndex == lockingStarSelectionIndex) ? "> " : "  ");
    display.print(lockingStarOptions[optionIndex].name);
    display.setCursor(96, y);
    int altDeg = static_cast<int>(llround(lockingStarOptions[optionIndex].altDeg));
    display.printf("%+02d", altDeg);
    display.write(kDegreeSymbol);
  }

  display.setCursor(0, config::OLED_HEIGHT - kLineHeight);
  display.print("Enc=Goto Joy=Skip");
}

void drawLockingStarRefine() {
  display.setCursor(0, 12);
  display.print("Refine alignment");
  display.setCursor(0, 20);
  if (lockingStarPendingName.isEmpty()) {
    display.print("Selected star");
  } else {
    display.print(lockingStarPendingName);
  }
  double azError = 0.0;
  double altError = 0.0;
  if (computeLockingStarErrors(azError, altError)) {
    display.setCursor(0, 32);
    display.printf("dAz=%+.2f", azError);
    display.write(kDegreeSymbol);
    display.setCursor(0, 40);
    display.printf("dAlt=%+.2f", altError);
    display.write(kDegreeSymbol);
  } else {
    display.setCursor(0, 32);
    display.print("Center star");
    display.setCursor(0, 40);
    display.print("Use joystick");
  }
  display.setCursor(0, 52);
  display.print("Enc=Save Joy=Skip");
}

void drawStatusMenuPrompt() {
  int footerY = config::OLED_HEIGHT - kLineHeight;
  display.fillRect(0, footerY, config::OLED_WIDTH, kLineHeight, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(0, footerY);
  display.print("Menu");
  display.setTextColor(SSD1306_WHITE);
}

void drawMainMenu() {
  int visible = computeVisibleRows(0, 0);
  if (visible <= 0) {
    return;
  }
  int start = std::clamp(mainMenuScroll, 0, static_cast<int>(kMainMenuCount) - 1);
  int rows = std::min(visible, static_cast<int>(kMainMenuCount));
  for (int row = 0; row < rows; ++row) {
    int index = (start + row) % static_cast<int>(kMainMenuCount);
    int y = lineY(0, row);
    bool selected = index == mainMenuIndex;
    if (selected) {
      display.fillRect(0, y, config::OLED_WIDTH, kLineHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    display.print(kMainMenuItems[index]);
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
  }
}

void drawPolarAlignMenu() {
  display.setCursor(0, 16);
  display.print("Polar Align");
  display.setCursor(0, 24);
  if (orientationModel.hasCalibration()) {
    display.printf("dAz=%+.2f%c dAlt=%+.2f%c", orientationModel.azBias(), kDegreeSymbol,
                  orientationModel.altBias(), kDegreeSymbol);
  } else {
    display.print("No refinements");
  }
  constexpr int kListTop = 32;
  constexpr int kFooterHeight = 0;
  int visible = computeVisibleRows(kListTop, kFooterHeight);
  if (visible <= 0) {
    return;
  }
  ensureSelectionVisible(polarAlignMenuScroll, polarAlignMenuIndex, visible,
                         kPolarAlignMenuCount);
  int rows = std::min(visible, static_cast<int>(kPolarAlignMenuCount));
  for (int row = 0; row < rows; ++row) {
    int index = polarAlignMenuScroll + row;
    if (index >= static_cast<int>(kPolarAlignMenuCount)) {
      break;
    }
    int y = lineY(kListTop, row);
    bool selected = index == polarAlignMenuIndex;
    if (selected) {
      display.fillRect(0, y, config::OLED_WIDTH, kLineHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    display.print(kPolarAlignMenuItems[index]);
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
  }
}

void drawSetupMenu() {
  display.setCursor(0, 16);
  display.print("Setup");
  constexpr int kListTop = 24;
  constexpr int kFooterHeight = kLineHeight;
  int visible = computeVisibleRows(kListTop, kFooterHeight);
  if (visible <= 0) {
    return;
  }
  int start = std::clamp(setupMenuScroll, 0, static_cast<int>(kSetupMenuCount) - 1);
  int rows = std::min(visible, static_cast<int>(kSetupMenuCount));
  for (int row = 0; row < rows; ++row) {
    int index = (start + row) % static_cast<int>(kSetupMenuCount);
    int y = lineY(kListTop, row);
    bool selected = index == setupMenuIndex;
    if (selected) {
      display.fillRect(0, y, config::OLED_WIDTH, kLineHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    display.print(kSetupMenuItems[index]);
    if (index == kSetupMenuWifiIndex) {
      if (!wifi_ota::credentialsConfigured()) {
        display.print(": NoCfg");
      } else if (wifi_ota::isEnabled()) {
        display.print(wifi_ota::isConnected() ? ": On" : ": Conn");
      } else {
        display.print(": Off");
      }
    } else if (index == kSetupMenuWifiApIndex) {
      if (!stellarium_link::accessPointActive()) {
        display.print(": Off");
      } else if (stellarium_link::clientConnected()) {
        display.print(": Conn");
      } else {
        display.print(": On");
      }
    } else if (index == kSetupMenuStellariumIndex) {
      if (stellarium_link::clientConnected()) {
        display.print(": Active");
      } else if (stellarium_link::accessPointActive()) {
        display.print(": Waiting");
      } else {
        display.print(": Idle");
      }
    }
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
  }
  int footerY = config::OLED_HEIGHT - kFooterHeight;
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, footerY);
  display.print("Joy=Next Enc=Select");
}

void drawRtcEditor() {
  display.setCursor(0, 12);
  display.print("RTC Setup");
  constexpr int kListTop = 20;
  constexpr int kFooterHeight = kLineHeight;
  int visibleRows = computeVisibleRows(kListTop, kFooterHeight);
  if (visibleRows <= 0) {
    return;
  }
  ensureSelectionVisible(rtcEditScroll, rtcEdit.fieldIndex, visibleRows, kRtcFieldCount);

  const char* labels[] = {"Year", "Month", "Day", "Hour", "Min", "Sec"};
  int values[] = {rtcEdit.year, rtcEdit.month, rtcEdit.day,
                  rtcEdit.hour, rtcEdit.minute, rtcEdit.second};
  const char* dstLabels[] = {"Off", "On", "Auto"};

  for (int row = 0; row < visibleRows; ++row) {
    int index = rtcEditScroll + row;
    if (index >= kRtcFieldCount) {
      break;
    }
    int y = lineY(kListTop, row);
    bool selected = index == rtcEdit.fieldIndex;
    if (selected) {
      display.fillRect(0, y, config::OLED_WIDTH, kLineHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    if (index < 6) {
      display.printf("%s: %02d", labels[index], values[index]);
    } else if (index == 6) {
      int dstIndex = std::clamp(static_cast<int>(rtcEdit.dstMode), 0, 2);
      display.printf("DST: %s", dstLabels[dstIndex]);
    } else {
      display.print(rtcEdit.actionIndex == 0 ? "Save" : "Back");
    }
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
  }

  int footerY = config::OLED_HEIGHT - kFooterHeight;
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, footerY);
  display.print("Enc=Next/Conf Joy=Cancel");
}

void drawLocationSetup() {
  display.setCursor(0, 12);
  display.print("Location");
  int y = 24;
  auto drawRow = [&](int fieldIndex, const String& text) {
    bool selected = locationEdit.fieldIndex == fieldIndex;
    if (selected) {
      display.fillRect(0, y, config::OLED_WIDTH, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    display.print(text);
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
    y += 8;
  };

  char buffer[24];
  snprintf(buffer, sizeof(buffer), "Lat: %+07.3f%c", locationEdit.latitudeDeg, kDegreeSymbol);
  drawRow(0, buffer);
  snprintf(buffer, sizeof(buffer), "Lon: %+08.3f%c", locationEdit.longitudeDeg, kDegreeSymbol);
  drawRow(1, buffer);

  int tz = locationEdit.timezoneMinutes;
  int absTz = abs(tz);
  int tzHours = absTz / 60;
  int tzMinutes = absTz % 60;
  char sign = tz >= 0 ? '+' : '-';
  snprintf(buffer, sizeof(buffer), "TZ: %c%02d:%02d", sign, tzHours, tzMinutes);
  drawRow(2, buffer);

  bool actionSelected = locationEdit.fieldIndex == kLocationFieldCount - 1;
  if (actionSelected) {
    display.fillRect(0, y, config::OLED_WIDTH, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(0, y);
  display.print(locationEdit.actionIndex == 0 ? "Save" : "Back");
  if (actionSelected) {
    display.setTextColor(SSD1306_WHITE);
  }

  display.setCursor(0, 60);
  display.print("Enc=Next/Conf Joy=Cancel");
}

void drawCatalogTypeMenu() {
  display.setCursor(0, 10);
  display.print("Catalog Types");
  size_t typeCount = catalog::typeGroupCount();
  if (typeCount == 0) {
    display.setCursor(0, 20);
    display.print("No categories");
    display.setCursor(0, 28);
    display.print("Joy=Back");
    return;
  }
  int total = static_cast<int>(typeCount);
  while (catalogTypeIndex < 0) catalogTypeIndex += total;
  while (catalogTypeIndex >= total) catalogTypeIndex -= total;

  CatalogTypeSummary summary{};
  if (!catalog::getTypeSummary(static_cast<size_t>(catalogTypeIndex), summary)) {
    display.setCursor(0, 20);
    display.print("Invalid type");
    return;
  }

  display.setCursor(0, 20);
  display.print(summary.name);
  display.setCursor(0, 28);
  display.printf("(%d/%d)", catalogTypeIndex + 1, total);
  display.setCursor(0, 36);
  display.printf("Objects: %u", static_cast<unsigned>(summary.objectCount));
  display.setCursor(0, 52);
  display.print("Enc=Open Joy=Back");
}

void drawCatalogItemList() {
  CatalogTypeSummary summary{};
  if (!catalog::getTypeSummary(static_cast<size_t>(catalogTypeIndex), summary) ||
      summary.objectCount == 0) {
    display.setCursor(0, 10);
    display.print("Catalog");
    display.setCursor(0, 20);
    display.print("No entries");
    display.setCursor(0, 52);
    display.print("Enc=Back Joy=Types");
    return;
  }

  int localCount = static_cast<int>(summary.objectCount);
  while (catalogTypeObjectIndex < 0) catalogTypeObjectIndex += localCount;
  while (catalogTypeObjectIndex >= localCount) catalogTypeObjectIndex -= localCount;

  int top = 20;
  int bottomMargin = kLineHeight;
  int visibleRows = computeVisibleRows(top, bottomMargin);
  ensureSelectionVisible(catalogItemScroll, catalogTypeObjectIndex, visibleRows, summary.objectCount);

  display.setCursor(0, 10);
  display.print(summary.name);
  display.setCursor(config::OLED_WIDTH - 36, 10);
  display.printf("%d/%d", catalogTypeObjectIndex + 1, localCount);

  for (int row = 0; row < visibleRows; ++row) {
    int localIndex = catalogItemScroll + row;
    if (localIndex >= localCount) {
      break;
    }
    bool selected = (localIndex == catalogTypeObjectIndex);
    int y = lineY(top, row);
    display.fillRect(0, y, config::OLED_WIDTH, kLineHeight, selected ? SSD1306_WHITE : SSD1306_BLACK);
    display.setTextColor(selected ? SSD1306_BLACK : SSD1306_WHITE);

    size_t globalIndex = 0;
    String label = "(invalid)";
    if (catalog::getTypeObjectIndex(static_cast<size_t>(catalogTypeIndex),
                                    static_cast<size_t>(localIndex), globalIndex)) {
      const CatalogObject* object = catalog::get(globalIndex);
      if (object) {
        label = sanitizeForDisplay(object->name);
        if (selected) {
          catalogIndex = static_cast<int>(globalIndex);
        }
      }
    }

    display.setCursor(0, y);
    display.print(label);
  }

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, config::OLED_HEIGHT - kLineHeight);
  display.print("Enc=Info Joy=Types");
}

void drawCatalogItemDetail() {
  CatalogTypeSummary summary{};
  if (!catalog::getTypeSummary(static_cast<size_t>(catalogTypeIndex), summary) ||
      summary.objectCount == 0) {
    display.setCursor(0, 10);
    display.print("Catalog");
    display.setCursor(0, 20);
    display.print("No entries");
    return;
  }

  int localCount = static_cast<int>(summary.objectCount);
  while (catalogTypeObjectIndex < 0) catalogTypeObjectIndex += localCount;
  while (catalogTypeObjectIndex >= localCount) catalogTypeObjectIndex -= localCount;

  if (catalogDetailMenuIndex < 0 || catalogDetailMenuIndex >= kCatalogDetailMenuCount) {
    catalogDetailMenuIndex = 0;
  }

  size_t globalIndex = 0;
  if (!catalog::getTypeObjectIndex(static_cast<size_t>(catalogTypeIndex),
                                   static_cast<size_t>(catalogTypeObjectIndex), globalIndex)) {
    display.setCursor(0, 10);
    display.print("Catalog");
    display.setCursor(0, 20);
    display.print("Invalid entry");
    return;
  }

  catalogIndex = static_cast<int>(globalIndex);
  const CatalogObject* object = catalog::get(globalIndex);
  if (!object) {
    display.setCursor(0, 20);
    display.print("Invalid entry");
    return;
  }

  systemState.selectedCatalogIndex = catalogIndex;
  systemState.selectedCatalogTypeIndex = catalogTypeIndex;

  display.setCursor(0, 10);
  display.print(summary.name);
  display.setCursor(90, 10);
  display.printf("%d/%d", catalogTypeObjectIndex + 1, localCount);

  DateTime now = currentDateTime();
  double ra;
  double dec;
  getObjectRaDecAt(*object, now, 0.0, ra, dec, nullptr);
  char raBuffer[24];
  char decBuffer[24];
  formatRa(ra, raBuffer, sizeof(raBuffer));
  formatDec(dec, decBuffer, sizeof(decBuffer));
  double azDeg = 0.0;
  double altDeg = -90.0;
  bool above = raDecToAltAz(now, ra, dec, azDeg, altDeg);
  String objectName = sanitizeForDisplay(object->name);
  String objectType = sanitizeForDisplay(object->type);
  String objectCode = sanitizeForDisplay(object->code);
  display.setCursor(0, 20);
  display.print(objectName);
  display.setCursor(0, 28);
  display.print(objectType);
  if (objectCode.length() > 0) {
    display.print(" / ");
    display.print(objectCode);
  }
  display.setCursor(0, 36);
  display.print("RA: ");
  display.print(raBuffer);
  display.setCursor(0, 44);
  display.print("Dec: ");
  display.print(decBuffer);
  display.setCursor(0, 52);
  display.printf("Alt:%+.1f%c Mag: %.1f", altDeg, kDegreeSymbol, object->magnitude);
  if (!above) {
    display.print(" (below)");
  }

  int menuY = config::OLED_HEIGHT - kLineHeight;
  const char* labels[kCatalogDetailMenuCount] = {"Goto", "Back"};
  for (int i = 0; i < kCatalogDetailMenuCount; ++i) {
    bool selected = (catalogDetailMenuIndex == i);
    int x = (config::OLED_WIDTH / kCatalogDetailMenuCount) * i;
    int width = (i == kCatalogDetailMenuCount - 1) ? (config::OLED_WIDTH - x)
                                                   : (config::OLED_WIDTH / kCatalogDetailMenuCount);
    display.fillRect(x, menuY, width, kLineHeight, SSD1306_BLACK);
    if (catalogDetailSelectingAction && selected) {
      display.fillRect(x, menuY, width, kLineHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.drawRect(x, menuY, width, kLineHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(x + 2, menuY);
    display.print(labels[i]);
    if (catalogDetailSelectingAction && selected) {
      display.setTextColor(SSD1306_WHITE);
    }
  }
  display.setTextColor(SSD1306_WHITE);
}

void drawAxisOrientationSetup() {
  display.setCursor(0, 0);
  display.print("Axis Setup");
  int joyLabelX = config::OLED_WIDTH - 6 * 10;  // Rough width for "Joy=Cancel"
  if (joyLabelX < 0) {
    joyLabelX = 0;
  }
  display.setCursor(joyLabelX, 0);
  display.print("Joy=Cancel");

  int y = kLineHeight;
  auto drawOption = [&](int index, const char* text) {
    bool selected = axisOrientationState.fieldIndex == index;
    if (selected) {
      display.fillRect(0, y, config::OLED_WIDTH, kLineHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    display.print(text);
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
    y += kLineHeight;
  };

  char buffer[32];
  const char* xTarget = axisOrientationState.joystickSwapAxes ? "Alt" : "Az";
  const char* yTarget = axisOrientationState.joystickSwapAxes ? "Az" : "Alt";
  snprintf(buffer, sizeof(buffer), "Axes: X->%s Y->%s", xTarget, yTarget);
  drawOption(0, buffer);

  drawOption(1, axisOrientationState.joystickInvertAz ? "Joy Az Dir: Reverse"
                                                     : "Joy Az Dir: Normal");
  drawOption(2, axisOrientationState.joystickInvertAlt ? "Joy Alt Dir: Reverse"
                                                      : "Joy Alt Dir: Normal");
  drawOption(3, axisOrientationState.motorInvertAz ? "Motor Az Dir: Reverse"
                                                  : "Motor Az Dir: Normal");
  drawOption(4, axisOrientationState.motorInvertAlt ? "Motor Alt Dir: Reverse"
                                                   : "Motor Alt Dir: Normal");

  bool actionSelected = axisOrientationState.fieldIndex == kAxisOrientationFieldCount - 1;
  if (actionSelected) {
    display.fillRect(0, y, config::OLED_WIDTH, kLineHeight, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(0, y);
  display.print(axisOrientationState.actionIndex == 0 ? "Save" : "Back");
  if (actionSelected) {
    display.setTextColor(SSD1306_WHITE);
  }

  int helpY = config::OLED_HEIGHT - kLineHeight;
  if (helpY < 0) {
    helpY = 0;
  }
  display.setCursor(0, helpY);
  display.print("Press=Next Rot=Toggle");
}

void drawAxisCalibration() {
  display.setCursor(0, 12);
  display.print("Axis Cal");
  const char* steps[] = {
      "Set Az 0deg, enc",
      "Rotate +90deg, enc",
      "Set Alt 0deg, enc",
      "Rotate +45deg, enc"};
  int index = std::min(axisCal.step, 3);
  display.setCursor(0, 24);
  display.print(steps[index]);
}

void drawSpeedProfileSetup() {
  display.setCursor(0, 12);
  display.print(speedProfileState.mode == SpeedEditMode::Goto ? "Goto Speed"
                                                             : "Pan Speed");
  const char* labels[] = {"Max [deg/s]", "Accel [deg/s2]", "Decel [deg/s2]"};
  float values[] = {speedProfileState.maxSpeed, speedProfileState.acceleration,
                    speedProfileState.deceleration};
  int y = 24;
  for (int i = 0; i < 3; ++i) {
    bool selected = speedProfileState.fieldIndex == i;
    if (selected) {
      display.fillRect(0, y, config::OLED_WIDTH, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, y);
    display.printf("%s: %4.1f", labels[i], values[i]);
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
    y += 8;
  }
  bool actionSelected = speedProfileState.fieldIndex == kSpeedProfileFieldCount - 1;
  if (actionSelected) {
    display.fillRect(0, y, config::OLED_WIDTH, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(0, y);
  display.print(speedProfileState.actionIndex == 0 ? "Save" : "Back");
  if (actionSelected) {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(0, 60);
  display.print("Enc=Next/Conf Joy=Cancel");
}

void drawBacklashCalibration() {
  display.setCursor(0, 12);
  display.print("Backlash Cal");
  const char* prompts[] = {"Az fwd pos, enc", "Az reverse, enc", "Alt fwd pos, enc", "Alt reverse, enc", "Done"};
  int idx = std::min(backlashState.step, 4);
  display.setCursor(0, 24);
  display.print(prompts[idx]);
  display.setCursor(0, 40);
  display.print("Use joy to move");
  display.setCursor(0, 56);
  display.print("Joy btn = abort");
}

void drawGotoCoordinateEntry() {
  display.setCursor(0, 12);
  display.print("Goto RA/Dec");

  auto drawSegment = [&](int fieldIndex, int x, int y, int width, const char* text) {
    bool selected = gotoCoordinateState.fieldIndex == fieldIndex;
    if (selected) {
      display.fillRect(x, y, width, 8, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    }
    display.setCursor(x, y);
    display.print(text);
    if (selected) {
      display.setTextColor(SSD1306_WHITE);
    }
  };

  int yRa = 24;
  display.setCursor(0, yRa);
  display.print("RA");
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "%02dh", gotoCoordinateState.raHours);
  drawSegment(0, 24, yRa, 24, buffer);
  snprintf(buffer, sizeof(buffer), "%02dm", gotoCoordinateState.raMinutes);
  drawSegment(1, 52, yRa, 24, buffer);
  snprintf(buffer, sizeof(buffer), "%02ds", gotoCoordinateState.raSeconds);
  drawSegment(2, 80, yRa, 24, buffer);

  int yDec = 36;
  display.setCursor(0, yDec);
  display.print("Dec");
  snprintf(buffer, sizeof(buffer), "%c", gotoCoordinateState.decNegative ? '-' : '+');
  drawSegment(3, 24, yDec, 12, buffer);
  snprintf(buffer, sizeof(buffer), "%02d%c", gotoCoordinateState.decDegrees, kDegreeSymbol);
  drawSegment(4, 40, yDec, 28, buffer);
  snprintf(buffer, sizeof(buffer), "%02d'", gotoCoordinateState.decArcMinutes);
  drawSegment(5, 70, yDec, 24, buffer);
  snprintf(buffer, sizeof(buffer), "%02d\"", gotoCoordinateState.decArcSeconds);
  drawSegment(6, 98, yDec, 24, buffer);

  int yBack = 48;
  drawSegment(7, 0, yBack, 40, "Back");

  display.setCursor(0, 60);
  display.print("Joy=Next Enc=Go/Back");
}

void enterLocationSetup() {
  const SystemConfig& config = storage::getConfig();
  locationEdit.latitudeDeg = config.observerLatitudeDeg;
  locationEdit.longitudeDeg = config.observerLongitudeDeg;
  locationEdit.timezoneMinutes = config.timezoneOffsetMinutes;
  locationEdit.fieldIndex = 0;
  locationEdit.actionIndex = 0;
  setUiState(UiState::LocationSetup);
}

void enterAxisOrientationSetup() {
  const SystemConfig& config = storage::getConfig();
  axisOrientationState.joystickSwapAxes = config.joystickSwapAxes != 0;
  axisOrientationState.joystickInvertAz = config.joystickInvertAz != 0;
  axisOrientationState.joystickInvertAlt = config.joystickInvertAlt != 0;
  axisOrientationState.motorInvertAz = config.motorInvertAz != 0;
  axisOrientationState.motorInvertAlt = config.motorInvertAlt != 0;
  axisOrientationState.fieldIndex = 0;
  axisOrientationState.actionIndex = 0;
  setUiState(UiState::AxisOrientation);
}

void enterGotoSpeedSetup() {
  const GotoProfile& profile = storage::getConfig().gotoProfile;
  speedProfileState.maxSpeed = profile.maxSpeedDegPerSec;
  speedProfileState.acceleration = profile.accelerationDegPerSec2;
  speedProfileState.deceleration = profile.decelerationDegPerSec2;
  speedProfileState.fieldIndex = 0;
  speedProfileState.mode = SpeedEditMode::Goto;
  speedProfileState.actionIndex = 0;
  setUiState(UiState::GotoSpeed);
}

void enterPanningSpeedSetup() {
  const GotoProfile& profile = storage::getConfig().panningProfile;
  speedProfileState.maxSpeed = profile.maxSpeedDegPerSec;
  speedProfileState.acceleration = profile.accelerationDegPerSec2;
  speedProfileState.deceleration = profile.decelerationDegPerSec2;
  speedProfileState.fieldIndex = 0;
  speedProfileState.mode = SpeedEditMode::Panning;
  speedProfileState.actionIndex = 0;
  setUiState(UiState::PanningSpeed);
}

void handleSpeedProfileInput(int delta) {
  if (delta != 0) {
    if (speedProfileState.fieldIndex < kSpeedProfileFieldCount - 1) {
      constexpr float step = 0.1f;
      switch (speedProfileState.fieldIndex) {
        case 0:
          speedProfileState.maxSpeed =
              std::clamp(speedProfileState.maxSpeed + delta * step, 0.5f, 20.0f);
          break;
        case 1:
          speedProfileState.acceleration =
              std::clamp(speedProfileState.acceleration + delta * step, 0.1f, 20.0f);
          break;
        case 2:
          speedProfileState.deceleration =
              std::clamp(speedProfileState.deceleration + delta * step, 0.1f, 20.0f);
          break;
      }
    } else {
      int step = delta > 0 ? 1 : -1;
      speedProfileState.actionIndex = (speedProfileState.actionIndex + step + 2) % 2;
    }
  }
  if (input::consumeJoystickPress()) {
    setUiState(UiState::SetupMenu);
    showInfo(speedProfileState.mode == SpeedEditMode::Goto ? "Goto canceled"
                                                           : "Pan canceled");
    return;
  }
  if (input::consumeEncoderClick()) {
    if (speedProfileState.fieldIndex < kSpeedProfileFieldCount - 1) {
      ++speedProfileState.fieldIndex;
      if (speedProfileState.fieldIndex == kSpeedProfileFieldCount - 1) {
        speedProfileState.actionIndex = 0;
      }
      return;
    }
    if (speedProfileState.actionIndex == 0) {
      GotoProfile profile{speedProfileState.maxSpeed,
                          speedProfileState.acceleration,
                          speedProfileState.deceleration};
      if (speedProfileState.mode == SpeedEditMode::Goto) {
        storage::setGotoProfile(profile);
        showInfo("Goto saved");
      } else {
        storage::setPanningProfile(profile);
        showInfo("Pan saved");
      }
    } else {
      showInfo(speedProfileState.mode == SpeedEditMode::Goto ? "Goto unchanged"
                                                            : "Pan unchanged");
    }
    setUiState(UiState::SetupMenu);
  }
}

void handleAxisOrientationInput(int delta) {
  if (delta != 0) {
    if (axisOrientationState.fieldIndex < kAxisOrientationFieldCount - 1) {
      switch (axisOrientationState.fieldIndex) {
        case 0:
          axisOrientationState.joystickSwapAxes = !axisOrientationState.joystickSwapAxes;
          break;
        case 1:
          axisOrientationState.joystickInvertAz = !axisOrientationState.joystickInvertAz;
          break;
        case 2:
          axisOrientationState.joystickInvertAlt = !axisOrientationState.joystickInvertAlt;
          break;
        case 3:
          axisOrientationState.motorInvertAz = !axisOrientationState.motorInvertAz;
          break;
        case 4:
          axisOrientationState.motorInvertAlt = !axisOrientationState.motorInvertAlt;
          break;
      }
    } else {
      int step = delta > 0 ? 1 : -1;
      axisOrientationState.actionIndex = (axisOrientationState.actionIndex + step + 2) % 2;
    }
  }

  if (input::consumeJoystickPress()) {
    setUiState(UiState::SetupMenu);
    showInfo("Axis canceled");
    return;
  }

  if (input::consumeEncoderClick()) {
    if (axisOrientationState.fieldIndex < kAxisOrientationFieldCount - 1) {
      ++axisOrientationState.fieldIndex;
      if (axisOrientationState.fieldIndex == kAxisOrientationFieldCount - 1) {
        axisOrientationState.actionIndex = 0;
      }
      return;
    }

    if (axisOrientationState.actionIndex == 0) {
      storage::setJoystickOrientation(axisOrientationState.joystickSwapAxes,
                                      axisOrientationState.joystickInvertAz,
                                      axisOrientationState.joystickInvertAlt);
      storage::setMotorInversion(axisOrientationState.motorInvertAz,
                                 axisOrientationState.motorInvertAlt);
      bool applied = motion::setMotorInversion(axisOrientationState.motorInvertAz,
                                               axisOrientationState.motorInvertAlt);
      showInfo(applied ? "Axis saved" : "Axis pending");
    } else {
      showInfo("Axis unchanged");
    }
    setUiState(UiState::SetupMenu);
  }
}

void handleLocationInput(int delta) {
  if (delta != 0) {
    if (locationEdit.fieldIndex < kLocationFieldCount - 1) {
      switch (locationEdit.fieldIndex) {
        case 0:
          locationEdit.latitudeDeg =
              std::clamp(locationEdit.latitudeDeg + delta * 0.01, -90.0, 90.0);
          break;
        case 1:
          locationEdit.longitudeDeg =
              std::clamp(locationEdit.longitudeDeg + delta * 0.01, -180.0, 180.0);
          break;
        case 2: {
          int stepMinutes = delta * 15;
          int32_t newTimezone = locationEdit.timezoneMinutes + stepMinutes;
          locationEdit.timezoneMinutes =
              std::clamp<int32_t>(newTimezone, -720, 840);
          break;
        }
      }
    } else {
      int step = delta > 0 ? 1 : -1;
      locationEdit.actionIndex = (locationEdit.actionIndex + step + 2) % 2;
    }
  }
  if (input::consumeJoystickPress()) {
    setUiState(UiState::SetupMenu);
    showInfo("Location canceled");
    return;
  }
  if (input::consumeEncoderClick()) {
    if (locationEdit.fieldIndex < kLocationFieldCount - 1) {
      ++locationEdit.fieldIndex;
      if (locationEdit.fieldIndex == kLocationFieldCount - 1) {
        locationEdit.actionIndex = 0;
      }
      return;
    }
    if (locationEdit.actionIndex == 0) {
      storage::setObserverLocation(locationEdit.latitudeDeg, locationEdit.longitudeDeg,
                                   locationEdit.timezoneMinutes);
      showInfo("Location saved");
    } else {
      showInfo("Location unchanged");
    }
    setUiState(UiState::SetupMenu);
  }
}

void enterGotoCoordinateEntry() {
  double ra = manualGotoRaHours;
  double dec = manualGotoDecDegrees;
  if (systemState.selectedCatalogIndex >= 0 &&
      systemState.selectedCatalogIndex < static_cast<int>(catalog::size())) {
    const CatalogObject* object = catalog::get(static_cast<size_t>(systemState.selectedCatalogIndex));
    if (object) {
      ra = object->raHours;
      dec = object->decDegrees;
    }
  } else if (tracking.active) {
    ra = tracking.targetRaHours;
    dec = tracking.targetDecDegrees;
  }

  double normalizedRa = fmod(ra, 24.0);
  if (normalizedRa < 0.0) normalizedRa += 24.0;
  int hours = static_cast<int>(floor(normalizedRa));
  double raMinutesFloat = (normalizedRa - hours) * 60.0;
  int minutes = static_cast<int>(floor(raMinutesFloat));
  double raSecondsFloat = (raMinutesFloat - minutes) * 60.0;
  int seconds = static_cast<int>(llround(raSecondsFloat));
  if (seconds >= 60) {
    seconds -= 60;
    minutes += 1;
  }
  if (minutes >= 60) {
    minutes -= 60;
    hours = (hours + 1) % 24;
  }

  bool negative = dec < 0.0;
  double absDec = fabs(dec);
  if (absDec > 90.0) absDec = 90.0;
  int degrees = static_cast<int>(floor(absDec));
  double decMinutesFloat = (absDec - degrees) * 60.0;
  int decMinutes = static_cast<int>(floor(decMinutesFloat));
  double decSecondsFloat = (decMinutesFloat - decMinutes) * 60.0;
  int decSeconds = static_cast<int>(llround(decSecondsFloat));
  if (decSeconds >= 60) {
    decSeconds -= 60;
    decMinutes += 1;
  }
  if (decMinutes >= 60) {
    decMinutes -= 60;
    degrees += 1;
  }
  if (degrees > 90) {
    degrees = 90;
  }
  if (degrees == 90) {
    decMinutes = 0;
    decSeconds = 0;
  }

  gotoCoordinateState.raHours = hours;
  gotoCoordinateState.raMinutes = minutes;
  gotoCoordinateState.raSeconds = seconds;
  gotoCoordinateState.decNegative = negative;
  gotoCoordinateState.decDegrees = degrees;
  gotoCoordinateState.decArcMinutes = decMinutes;
  gotoCoordinateState.decArcSeconds = decSeconds;
  gotoCoordinateState.fieldIndex = 0;
  systemState.menuMode = MenuMode::Goto;
  setUiState(UiState::GotoCoordinates);
}

void handleGotoCoordinateInput(int delta) {
  if (delta != 0) {
    switch (gotoCoordinateState.fieldIndex) {
      case 0: {
        int value = gotoCoordinateState.raHours + delta;
        value %= 24;
        if (value < 0) value += 24;
        gotoCoordinateState.raHours = value;
        break;
      }
      case 1: {
        int value = gotoCoordinateState.raMinutes + delta;
        while (value < 0) value += 60;
        while (value >= 60) value -= 60;
        gotoCoordinateState.raMinutes = value;
        break;
      }
      case 2: {
        int value = gotoCoordinateState.raSeconds + delta;
        while (value < 0) value += 60;
        while (value >= 60) value -= 60;
        gotoCoordinateState.raSeconds = value;
        break;
      }
      case 3:
        gotoCoordinateState.decNegative = !gotoCoordinateState.decNegative;
        break;
      case 4: {
        int value = gotoCoordinateState.decDegrees + delta;
        value = std::clamp(value, 0, 90);
        gotoCoordinateState.decDegrees = value;
        if (value == 90) {
          gotoCoordinateState.decArcMinutes = 0;
          gotoCoordinateState.decArcSeconds = 0;
        }
        break;
      }
      case 5: {
        int value = gotoCoordinateState.decArcMinutes + delta;
        while (value < 0) value += 60;
        while (value >= 60) value -= 60;
        gotoCoordinateState.decArcMinutes = value;
        if (gotoCoordinateState.decDegrees == 90) {
          gotoCoordinateState.decArcMinutes = 0;
          gotoCoordinateState.decArcSeconds = 0;
        }
        break;
      }
      case 6: {
        int value = gotoCoordinateState.decArcSeconds + delta;
        while (value < 0) value += 60;
        while (value >= 60) value -= 60;
        gotoCoordinateState.decArcSeconds = value;
        if (gotoCoordinateState.decDegrees == 90) {
          gotoCoordinateState.decArcSeconds = 0;
        }
        break;
      }
      default:
        break;
    }
  }

  if (input::consumeJoystickPress()) {
    gotoCoordinateState.fieldIndex = (gotoCoordinateState.fieldIndex + 1) % kGotoCoordinateFieldCount;
  }

  if (input::consumeEncoderClick()) {
    if (gotoCoordinateState.fieldIndex == kGotoCoordinateFieldCount - 1) {
      systemState.menuMode = MenuMode::Status;
      setUiState(UiState::MainMenu);
      return;
    }

    double ra = gotoCoordinateState.raHours + gotoCoordinateState.raMinutes / 60.0 +
                gotoCoordinateState.raSeconds / 3600.0;
    ra = fmod(ra, 24.0);
    if (ra < 0.0) ra += 24.0;
    double dec = gotoCoordinateState.decDegrees + gotoCoordinateState.decArcMinutes / 60.0 +
                 gotoCoordinateState.decArcSeconds / 3600.0;
    if (gotoCoordinateState.decNegative) dec = -dec;

    manualGotoRaHours = ra;
    manualGotoDecDegrees = dec;
    String label = makeRaDecLabel(ra, dec);
    if (startGotoToCoordinates(ra, dec, label)) {
      selectedObjectName = sanitizeForDisplay(label);
      gotoTargetName = sanitizeForDisplay(label);
      systemState.selectedCatalogIndex = -1;
      systemState.menuMode = MenuMode::Status;
      setUiState(UiState::StatusScreen);
    }
  }
}

void startBacklashCalibration() {
  backlashState = {0, 0, 0, 0, 0};
  setUiState(UiState::BacklashCalibration);
  showInfo("Az fwd pos");
}

void completeBacklashCalibration() {
  int32_t azSteps = static_cast<int32_t>(std::min<int64_t>(llabs(backlashState.azEnd - backlashState.azStart), INT32_MAX));
  int32_t altSteps = static_cast<int32_t>(std::min<int64_t>(llabs(backlashState.altEnd - backlashState.altStart), INT32_MAX));
  BacklashConfig config{azSteps, altSteps};
  storage::setBacklash(config);
  motion::setBacklash(config);
  showInfo("Backlash saved");
  setUiState(UiState::SetupMenu);
}

void handleBacklashCalibrationInput() {
  if (input::consumeJoystickPress()) {
    setUiState(UiState::SetupMenu);
    showInfo("Cal aborted");
    return;
  }
  bool select = input::consumeEncoderClick();
  if (!select) {
    return;
  }
  switch (backlashState.step) {
    case 0:
      backlashState.azStart = motion::getStepCount(Axis::Az);
      backlashState.step = 1;
      showInfo("Reverse AZ");
      break;
    case 1:
      backlashState.azEnd = motion::getStepCount(Axis::Az);
      backlashState.step = 2;
      showInfo("Set Alt pos");
      break;
    case 2:
      backlashState.altStart = motion::getStepCount(Axis::Alt);
      backlashState.step = 3;
      showInfo("Reverse ALT");
      break;
    case 3:
      backlashState.altEnd = motion::getStepCount(Axis::Alt);
      backlashState.step = 4;
      completeBacklashCalibration();
      break;
    default:
      setUiState(UiState::SetupMenu);
      break;
  }
}

void render() {
  MutexLock lock(i2cMutex);
  if (!lock.locked()) {
    return;
  }

  uint32_t now = millis();
  if (bootAnimationActive) {
    if (bootAnimationStopRequested && now >= bootAnimationMinEndMs) {
      bootAnimationActive = false;
      bootAnimationStopRequested = false;
    }
    if (bootAnimationActive) {
      drawBootAnimation(now);
      display.display();
      return;
    }
  }

  display.clearDisplay();

  bool showHeader = uiState != UiState::MainMenu;
  if (showHeader) {
    drawHeader();
  }

  String message;
  if (fetchInfoMessage(message)) {
    display.setCursor(0, 12);
    display.print(message);
    display.display();
    return;
  }

  switch (uiState) {
    case UiState::StatusScreen:
      drawStatus(false);
      drawStatusMenuPrompt();
      break;
    case UiState::StatusDetails:
      drawStatus(true);
      break;
    case UiState::StartupLockPrompt:
      drawStartupLockPrompt();
      break;
    case UiState::MainMenu:
      drawMainMenu();
      break;
    case UiState::PolarAlignMenu:
      drawPolarAlignMenu();
      break;
    case UiState::PolarAlign:
      drawStatus(false);
      display.setCursor(0, 36);
      display.print("Center Polaris");
      display.setCursor(0, 44);
      display.print("Enc=Confirm");
      display.setCursor(0, 52);
      display.print("Joy=Abort");
      break;
    case UiState::LockingStarMenu:
      drawLockingStarMenu();
      break;
    case UiState::LockingStarRefine:
      drawLockingStarRefine();
      break;
    case UiState::SetupMenu:
      drawSetupMenu();
      break;
    case UiState::SetRtc:
      drawRtcEditor();
      break;
    case UiState::LocationSetup:
      drawLocationSetup();
      break;
    case UiState::AxisOrientation:
      drawAxisOrientationSetup();
      break;
    case UiState::CatalogTypeBrowser:
      drawCatalogTypeMenu();
      break;
    case UiState::CatalogItemList:
      drawCatalogItemList();
      break;
    case UiState::CatalogItemDetail:
      drawCatalogItemDetail();
      break;
    case UiState::AxisCalibration:
      drawAxisCalibration();
      break;
    case UiState::GotoSpeed:
    case UiState::PanningSpeed:
      drawSpeedProfileSetup();
      break;
    case UiState::GotoCoordinates:
      drawGotoCoordinateEntry();
      break;
    case UiState::BacklashCalibration:
      drawBacklashCalibration();
      break;
  }

  display.display();
}

void displayTask(void*) {
  for (;;) {
    render();
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void enterPolarAlignMenu() {
  polarAlignMenuIndex = 0;
  polarAlignMenuScroll = 0;
  systemState.menuMode = MenuMode::PolarAlign;
  setUiState(UiState::PolarAlignMenu);
}

void enterSetupMenu() {
  setupMenuIndex = 0;
  setupMenuScroll = 0;
  setUiState(UiState::SetupMenu);
}

void enterRtcEditor() {
  DateTime now;
  const SystemConfig& config = storage::getConfig();
  if (rtcAvailable) {
    MutexLock lock(i2cMutex);
    if (lock.locked()) {
      time_t utcEpoch = rtc.now().unixtime();
      now = time_utils::applyTimezone(utcEpoch);
    } else if (config.lastRtcEpoch != 0) {
      now = time_utils::applyTimezone(static_cast<time_t>(config.lastRtcEpoch));
    } else {
      now = DateTime(2024, 1, 1, 0, 0, 0);
    }
  } else if (config.lastRtcEpoch != 0) {
    now = time_utils::applyTimezone(static_cast<time_t>(config.lastRtcEpoch));
  } else {
    now = DateTime(2024, 1, 1, 0, 0, 0);
  }
  rtcEdit = {now.year(),      now.month(),      now.day(),
             now.hour(),      now.minute(),     now.second(),
             storage::getConfig().dstMode, 0, 0};
  rtcEditScroll = 0;
  setUiState(UiState::SetRtc);
}

void applyRtcEdit() {
  DateTime updated(rtcEdit.year, rtcEdit.month, rtcEdit.day, rtcEdit.hour, rtcEdit.minute, rtcEdit.second);
  time_t utcEpoch = time_utils::toUtcEpoch(updated);
  if (rtcAvailable) {
    MutexLock lock(i2cMutex);
    if (lock.locked()) {
      rtc.adjust(DateTime(utcEpoch));
    }
  }
  storage::setDstMode(rtcEdit.dstMode);
  storage::setRtcEpoch(static_cast<uint32_t>(utcEpoch));
  showInfo("RTC updated");
  setUiState(UiState::SetupMenu);
}

void startJoystickCalibrationFlow() {
  showCalibrationStart();
  auto calibration = input::calibrateJoystick();
  input::setJoystickCalibration(calibration);
  storage::setJoystickCalibration(calibration);
  showCalibrationResult(calibration.centerX, calibration.centerY);
  setUiState(UiState::SetupMenu);
}

void resetAxisCalibrationState() {
  axisCal = {0, 0, 0, 0, 0};
  setUiState(UiState::AxisCalibration);
}

void completeAxisCalibration() {
  double azSpan = fabs(static_cast<double>(axisCal.azReference - axisCal.azZero));
  double altSpan = fabs(static_cast<double>(axisCal.altReference - axisCal.altZero));
  double stepsPerAzDegree = azSpan / 90.0;
  double stepsPerAltDegree = altSpan / 45.0;
  if (stepsPerAzDegree < 1.0 || stepsPerAltDegree < 1.0) {
    showInfo("Cal failed");
    resetAxisCalibrationState();
    return;
  }
  AxisCalibration calibration;
  calibration.stepsPerDegreeAz = stepsPerAzDegree;
  calibration.stepsPerDegreeAlt = stepsPerAltDegree;
  calibration.azHomeOffset = axisCal.azZero;
  calibration.altHomeOffset = axisCal.altZero;
  storage::setAxisCalibration(calibration);
  motion::applyCalibration(calibration);
  showInfo("Axes calibrated");
  setUiState(UiState::SetupMenu);
}

void handleAxisCalibrationClick() {
  switch (axisCal.step) {
    case 0:
      axisCal.azZero = motion::getStepCount(Axis::Az);
      axisCal.step = 1;
      showInfo("Rotate +90deg");
      break;
    case 1:
      axisCal.azReference = motion::getStepCount(Axis::Az);
      axisCal.step = 2;
      showInfo("Set Alt 0");
      break;
    case 2:
      axisCal.altZero = motion::getStepCount(Axis::Alt);
      axisCal.step = 3;
      showInfo("Rotate +45deg");
      break;
    case 3:
      axisCal.altReference = motion::getStepCount(Axis::Alt);
      axisCal.step = 4;
      completeAxisCalibration();
      break;
    default:
      break;
  }
}

AxisGotoRuntime initAxisRuntime(Axis axis, int64_t targetSteps) {
  AxisGotoRuntime runtime{};
  runtime.finalTarget = targetSteps;
  runtime.compensatedTarget = targetSteps;
  runtime.currentSpeed = 0.0;
  runtime.desiredDirection = 0;
  runtime.compensationPending = false;
  runtime.reachedFinalTarget = false;

  int64_t current = motion::getStepCount(axis);
  int64_t diff = targetSteps - current;
  if (diff == 0) {
    runtime.reachedFinalTarget = true;
    return runtime;
  }

  runtime.desiredDirection = diff >= 0 ? 1 : -1;
  int8_t lastDir = motion::getLastDirection(axis);
  int32_t backlash = motion::getBacklashSteps(axis);
  if (backlash > 0 && lastDir != 0 && lastDir != runtime.desiredDirection) {
    runtime.compensatedTarget = targetSteps + runtime.desiredDirection * backlash;
    runtime.compensationPending = true;
  }
  return runtime;
}

bool updateAxisGoto(Axis axis,
                    AxisGotoRuntime& runtime,
                    double dt,
                    const GotoProfileSteps& profile) {
  if (runtime.reachedFinalTarget) {
    motion::setGotoStepsPerSecond(axis, 0.0);
    return true;
  }

  int64_t current = motion::getStepCount(axis);
  int64_t error = runtime.compensatedTarget - current;
  double absError = fabs(static_cast<double>(error));
  double direction = (error >= 0) ? 1.0 : -1.0;

  double maxSpeed = std::max(axis == Axis::Az ? profile.maxSpeedAz : profile.maxSpeedAlt, 1.0);
  double accel = std::max(axis == Axis::Az ? profile.accelerationAz : profile.accelerationAlt, 1.0);
  double decel = std::max(axis == Axis::Az ? profile.decelerationAz : profile.decelerationAlt, 1.0);

  double speed = runtime.currentSpeed;
  double distanceToStop = (speed * speed) / (2.0 * decel);

  if (absError <= 1.0 && speed < 1.0) {
    motion::setGotoStepsPerSecond(axis, 0.0);
    if (runtime.compensationPending) {
      runtime.compensationPending = false;
      runtime.compensatedTarget = runtime.finalTarget;
      runtime.currentSpeed = 0.0;
      return false;
    }
    runtime.reachedFinalTarget = true;
    return true;
  }

  if (absError <= distanceToStop + 1.0) {
    speed -= decel * dt;
    if (speed < 0.0) speed = 0.0;
  } else {
    speed += accel * dt;
    if (speed > maxSpeed) speed = maxSpeed;
  }

  if (speed < 1.0 && absError > 2.0) {
    speed = std::min(maxSpeed, speed + accel * dt);
  }

  double commanded = speed * direction;
  motion::setGotoStepsPerSecond(axis, commanded);
  runtime.currentSpeed = speed;
  return false;
}

bool computeTargetAltAz(const CatalogObject& object,
                        const DateTime& start,
                        double secondsAhead,
                        double& raHours,
                        double& decDegrees,
                        double& azDeg,
                        double& altDeg,
                        DateTime& targetTime) {
  getObjectRaDecAt(object, start, secondsAhead, raHours, decDegrees, &targetTime);
  return raDecToAltAz(targetTime, raHours, decDegrees, azDeg, altDeg);
}

bool computeManualTarget(double raHours,
                         double decDegrees,
                         const DateTime& start,
                         double secondsAhead,
                         double& outRaHours,
                         double& outDecDegrees,
                         double& azDeg,
                         double& altDeg,
                         DateTime& targetTime) {
  targetTime = start + TimeSpan(0, 0, 0, static_cast<int32_t>(secondsAhead));
  outRaHours = raHours;
  outDecDegrees = decDegrees;
  return raDecToAltAz(targetTime, raHours, decDegrees, azDeg, altDeg);
}

template <typename ComputeFn>
bool planGotoTarget(const String& targetName, int targetCatalogIndex, ComputeFn computeTarget) {
  if (!systemState.polarAligned || !orientationKnown) {
    showInfo("Align first");
    return false;
  }
  const AxisCalibration& cal = storage::getConfig().axisCalibration;
  if (cal.stepsPerDegreeAz <= 0.0 || cal.stepsPerDegreeAlt <= 0.0) {
    showInfo("Calibrate axes");
    return false;
  }

  if (gotoRuntime.active) {
    abortGoto();
  }

  DateTime now = currentDateTime();
  double currentAz = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
  double currentAlt = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));

  double raNow;
  double decNow;
  double azNow;
  double altNow;
  DateTime timeNow;
  if (!computeTarget(now, 0.0, raNow, decNow, azNow, altNow, timeNow) ||
      altNow < motion::getMinAltitudeDegrees()) {
    showInfo("Below horizon");
    return false;
  }

  GotoProfileSteps profile = toProfileSteps(storage::getConfig().gotoProfile, cal);
  double commandAzNow = orientationModel.toPhysicalAz(azNow);
  double commandAltNow = orientationModel.toPhysicalAlt(altNow);
  commandAltNow =
      std::clamp(commandAltNow, motion::getMinAltitudeDegrees(), motion::getMaxAltitudeDegrees());
  double azDiffNow = shortestAngularDistance(currentAz, commandAzNow) * cal.stepsPerDegreeAz;
  double altDiffNow = (commandAltNow - currentAlt) * cal.stepsPerDegreeAlt;
  double durationAz =
      computeTravelTimeSteps(azDiffNow, profile.maxSpeedAz, profile.accelerationAz, profile.decelerationAz);
  double durationAlt =
      computeTravelTimeSteps(altDiffNow, profile.maxSpeedAlt, profile.accelerationAlt, profile.decelerationAlt);
  double estimatedDuration = std::max(durationAz, durationAlt) + 1.0;

  double raFuture;
  double decFuture;
  double azFuture;
  double altFuture;
  DateTime arrivalTime;
  if (!computeTarget(now, estimatedDuration, raFuture, decFuture, azFuture, altFuture, arrivalTime) ||
      altFuture < motion::getMinAltitudeDegrees()) {
    showInfo("Below horizon");
    return false;
  }

  int64_t currentAzSteps = motion::getStepCount(Axis::Az);
  int64_t currentAltSteps = motion::getStepCount(Axis::Alt);
  double commandAzFuture = orientationModel.toPhysicalAz(azFuture);
  double commandAltFuture = orientationModel.toPhysicalAlt(altFuture);
  commandAltFuture =
      std::clamp(commandAltFuture, motion::getMinAltitudeDegrees(), motion::getMaxAltitudeDegrees());
  int64_t targetAzSteps =
      currentAzSteps + static_cast<int64_t>(llround(shortestAngularDistance(currentAz, commandAzFuture) * cal.stepsPerDegreeAz));
  int64_t targetAltSteps = motion::altDegreesToSteps(commandAltFuture);

  gotoRuntime.active = true;
  gotoRuntime.az = initAxisRuntime(Axis::Az, targetAzSteps);
  gotoRuntime.alt = initAxisRuntime(Axis::Alt, targetAltSteps);
  gotoRuntime.profile = profile;
  double azDiffRemaining = static_cast<double>(targetAzSteps - currentAzSteps);
  double altDiffRemaining = static_cast<double>(targetAltSteps - currentAltSteps);
  double durationAzFuture =
      computeTravelTimeSteps(azDiffRemaining, profile.maxSpeedAz, profile.accelerationAz, profile.decelerationAz);
  double durationAltFuture =
      computeTravelTimeSteps(altDiffRemaining, profile.maxSpeedAlt, profile.accelerationAlt, profile.decelerationAlt);
  gotoRuntime.estimatedDurationSec = std::max(durationAzFuture, durationAltFuture) + 1.0;
  gotoRuntime.lastUpdateMs = millis();
  gotoRuntime.startTime = now;
  gotoRuntime.targetRaHours = raFuture;
  gotoRuntime.targetDecDegrees = decFuture;
  gotoRuntime.targetCatalogIndex = targetCatalogIndex;
  gotoRuntime.resumeTracking = true;

  systemState.gotoActive = true;
  systemState.azGotoTarget = targetAzSteps;
  systemState.altGotoTarget = targetAltSteps;
  gotoTargetName = sanitizeForDisplay(targetName);
  motion::clearGotoRates();
  stopTracking();
  showInfo("Goto started");
  return true;
}

void finalizeTrackingTarget(int catalogIndex,
                            double raHours,
                            double decDegrees,
                            double azDeg,
                            double altDeg) {
  tracking.active = true;
  tracking.targetCatalogIndex = catalogIndex;
  tracking.targetRaHours = raHours;
  tracking.targetDecDegrees = decDegrees;
  double physicalAz = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
  double physicalAlt = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));
  double skyAz = orientationModel.toSkyAz(physicalAz);
  double skyAlt = orientationModel.toSkyAlt(physicalAlt);
  tracking.offsetAzDeg = wrapAngle180(skyAz - azDeg);
  tracking.offsetAltDeg = skyAlt - altDeg;
  tracking.userAdjusting = false;
  systemState.trackingActive = true;
  motion::setTrackingEnabled(true);
}

bool startTrackingCurrentOrientation() {
  DateTime now = currentDateTime();
  double physicalAz = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
  double physicalAlt = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));
  double skyAz = orientationModel.toSkyAz(physicalAz);
  double skyAlt = orientationModel.toSkyAlt(physicalAlt);
  double raHours = 0.0;
  double decDegrees = 0.0;
  if (!altAzToRaDec(now, skyAz, skyAlt, raHours, decDegrees)) {
    return false;
  }
  double targetAz = 0.0;
  double targetAlt = 0.0;
  if (!raDecToAltAz(now, raHours, decDegrees, targetAz, targetAlt)) {
    return false;
  }
  finalizeTrackingTarget(-1, raHours, decDegrees, targetAz, targetAlt);
  return true;
}

void completeGotoSuccess() {
  motion::clearGotoRates();
  systemState.gotoActive = false;
  gotoRuntime.active = false;
  if (!gotoRuntime.resumeTracking) {
    showInfo("Parked");
    stopTracking();
    gotoRuntime.resumeTracking = true;
    return;
  }

  showInfo("Goto done");

  double azDeg = 0.0;
  double altDeg = 0.0;
  DateTime now = currentDateTime();
  double ra = gotoRuntime.targetRaHours;
  double dec = gotoRuntime.targetDecDegrees;
  raDecToAltAz(now, ra, dec, azDeg, altDeg);
  finalizeTrackingTarget(gotoRuntime.targetCatalogIndex, ra, dec, azDeg, altDeg);
  if (lockingStarFlowActive && lockingStarGotoInProgress &&
      gotoRuntime.targetCatalogIndex == lockingStarPendingCatalogIndex) {
    lockingStarGotoInProgress = false;
    lockingStarPendingRefine = true;
    lockingStarPendingRaHours = ra;
    lockingStarPendingDecDegrees = dec;
    setUiState(UiState::LockingStarRefine);
  }
}

void abortGoto() {
  motion::clearGotoRates();
  gotoRuntime.active = false;
  systemState.gotoActive = false;
  gotoRuntime.resumeTracking = true;
  stopTracking();
  if (lockingStarFlowActive && lockingStarGotoInProgress) {
    lockingStarGotoInProgress = false;
    lockingStarPendingRefine = false;
    lockingStarPendingCatalogIndex = -1;
    showLockingStarMenu();
  }
}

void abortGotoFromNetwork() { abortGoto(); }

void updateTracking() {
  if (gotoRuntime.active || systemState.gotoActive) {
    motion::setTrackingRates(0.0, 0.0);
    motion::setTrackingEnabled(false);
    return;
  }

  if (!tracking.active) {
    motion::setTrackingRates(0.0, 0.0);
    motion::setTrackingEnabled(false);
    systemState.trackingActive = false;
    return;
  }

  DateTime now = currentDateTime();
  double ra = tracking.targetRaHours;
  double dec = tracking.targetDecDegrees;
  if (tracking.targetCatalogIndex >= 0 &&
      tracking.targetCatalogIndex < static_cast<int>(catalog::size())) {
    const CatalogObject* object = catalog::get(static_cast<size_t>(tracking.targetCatalogIndex));
    if (object) {
      getObjectRaDecAt(*object, now, 0.0, ra, dec, nullptr);
    }
  }

  double azDeg = 0.0;
  double altDeg = 0.0;
  if (!raDecToAltAz(now, ra, dec, azDeg, altDeg)) {
    motion::setTrackingRates(0.0, 0.0);
    systemState.trackingActive = false;
    return;
  }

  double desiredAz = wrapAngle360(azDeg + tracking.offsetAzDeg);
  double desiredAlt = altDeg + tracking.offsetAltDeg;
  double desiredPhysicalAz = orientationModel.toPhysicalAz(desiredAz);
  double desiredPhysicalAlt = orientationModel.toPhysicalAlt(desiredAlt);
  desiredPhysicalAlt =
      std::clamp(desiredPhysicalAlt, motion::getMinAltitudeDegrees(), motion::getMaxAltitudeDegrees());
  double currentAz = motion::stepsToAzDegrees(motion::getStepCount(Axis::Az));
  double currentAlt = motion::stepsToAltDegrees(motion::getStepCount(Axis::Alt));
  double actualSkyAz = orientationModel.toSkyAz(currentAz);
  double actualSkyAlt = orientationModel.toSkyAlt(currentAlt);

  if (systemState.joystickActive) {
    tracking.userAdjusting = true;
  } else if (tracking.userAdjusting) {
    tracking.userAdjusting = false;
    tracking.offsetAzDeg = wrapAngle180(actualSkyAz - azDeg);
    tracking.offsetAltDeg = actualSkyAlt - altDeg;
    desiredAz = wrapAngle360(azDeg + tracking.offsetAzDeg);
    desiredAlt = altDeg + tracking.offsetAltDeg;
    desiredPhysicalAz = orientationModel.toPhysicalAz(desiredAz);
    desiredPhysicalAlt =
        orientationModel.toPhysicalAlt(desiredAlt);
    desiredPhysicalAlt = std::clamp(desiredPhysicalAlt, motion::getMinAltitudeDegrees(),
                                    motion::getMaxAltitudeDegrees());
  }

  double errorAz = shortestAngularDistance(currentAz, desiredPhysicalAz);
  double errorAlt = desiredPhysicalAlt - currentAlt;
  constexpr double kTrackingGain = 0.4;
  constexpr double kMaxTrackingSpeed = 3.0;
  double azRate = std::clamp(errorAz * kTrackingGain, -kMaxTrackingSpeed, kMaxTrackingSpeed);
  double altRate = std::clamp(errorAlt * kTrackingGain, -kMaxTrackingSpeed, kMaxTrackingSpeed);

  motion::setTrackingRates(azRate, altRate);
  motion::setTrackingEnabled(true);
  systemState.trackingActive = true;
}

void updateGoto() {
  if (!gotoRuntime.active) {
    if (systemState.gotoActive) {
      abortGoto();
    }
    updateTracking();
    return;
  }

  if (!systemState.gotoActive) {
    abortGoto();
    updateTracking();
    return;
  }

  uint32_t nowMs = millis();
  double dt = (nowMs - gotoRuntime.lastUpdateMs) / 1000.0;
  gotoRuntime.lastUpdateMs = nowMs;
  if (dt <= 0.0) {
    return;
  }

  bool azDone = updateAxisGoto(Axis::Az, gotoRuntime.az, dt, gotoRuntime.profile);
  bool altDone = updateAxisGoto(Axis::Alt, gotoRuntime.alt, dt, gotoRuntime.profile);

  if (azDone && altDone) {
    completeGotoSuccess();
  }
}

bool startGotoToObject(const CatalogObject& object, int catalogIndex) {
  auto compute = [&](const DateTime& start,
                     double secondsAhead,
                     double& raHours,
                     double& decDegrees,
                     double& azDeg,
                     double& altDeg,
                     DateTime& targetTime) {
    return computeTargetAltAz(object, start, secondsAhead, raHours, decDegrees, azDeg, altDeg, targetTime);
  };
  return planGotoTarget(object.name, catalogIndex, compute);
}

bool startGotoToCoordinates(double raHours, double decDegrees, const String& label) {
  auto compute = [&](const DateTime& start,
                     double secondsAhead,
                     double& outRa,
                     double& outDec,
                     double& azDeg,
                     double& altDeg,
                     DateTime& targetTime) {
    return computeManualTarget(raHours, decDegrees, start, secondsAhead, outRa, outDec, azDeg, altDeg, targetTime);
  };
  return planGotoTarget(label, -1, compute);
}

bool requestGotoFromNetwork(double raHours, double decDegrees, const String& label) {
  return startGotoToCoordinates(raHours, decDegrees, label);
}

bool startParkPosition() {
  const AxisCalibration& cal = storage::getConfig().axisCalibration;
  if (cal.stepsPerDegreeAz <= 0.0 || cal.stepsPerDegreeAlt <= 0.0) {
    showInfo("Calibrate axes");
    return false;
  }

  if (gotoRuntime.active) {
    abortGoto();
  }

  stopTracking();

  GotoProfileSteps profile = toProfileSteps(storage::getConfig().gotoProfile, cal);
  int64_t currentAzSteps = motion::getStepCount(Axis::Az);
  int64_t currentAltSteps = motion::getStepCount(Axis::Alt);
  int64_t targetAzSteps = currentAzSteps;
  int64_t targetAltSteps = motion::altDegreesToSteps(motion::getMaxAltitudeDegrees());

  gotoRuntime.active = true;
  gotoRuntime.az = initAxisRuntime(Axis::Az, targetAzSteps);
  gotoRuntime.alt = initAxisRuntime(Axis::Alt, targetAltSteps);
  gotoRuntime.profile = profile;
  double durationAz = computeTravelTimeSteps(static_cast<double>(targetAzSteps - currentAzSteps),
                                             profile.maxSpeedAz, profile.accelerationAz, profile.decelerationAz);
  double durationAlt = computeTravelTimeSteps(static_cast<double>(targetAltSteps - currentAltSteps),
                                              profile.maxSpeedAlt, profile.accelerationAlt, profile.decelerationAlt);
  gotoRuntime.estimatedDurationSec = std::max(durationAz, durationAlt) + 1.0;
  gotoRuntime.lastUpdateMs = millis();
  gotoRuntime.startTime = currentDateTime();
  gotoRuntime.targetRaHours = 0.0;
  gotoRuntime.targetDecDegrees = motion::getMaxAltitudeDegrees();
  gotoRuntime.targetCatalogIndex = -1;
  gotoRuntime.resumeTracking = false;

  systemState.gotoActive = true;
  systemState.azGotoTarget = targetAzSteps;
  systemState.altGotoTarget = targetAltSteps;
  gotoTargetName = sanitizeForDisplay("Park");
  motion::clearGotoRates();
  showInfo("Parking");
  return true;
}

void startGotoToSelected() {
  if (catalog::size() == 0 || systemState.selectedCatalogIndex < 0 ||
      systemState.selectedCatalogIndex >= static_cast<int>(catalog::size())) {
    showInfo("Select object");
    return;
  }
  const CatalogObject* object = catalog::get(static_cast<size_t>(systemState.selectedCatalogIndex));
  if (!object) {
    showInfo("Invalid object");
    return;
  }
  if (startGotoToObject(*object, systemState.selectedCatalogIndex)) {
    selectedObjectName = sanitizeForDisplay(object->name);
    gotoTargetName = sanitizeForDisplay(object->name);
  }
}

void handleMainMenuInput(int delta) {
  if (delta != 0) {
    mainMenuIndex += delta;
    while (mainMenuIndex < 0) mainMenuIndex += static_cast<int>(kMainMenuCount);
    while (mainMenuIndex >= static_cast<int>(kMainMenuCount))
      mainMenuIndex -= static_cast<int>(kMainMenuCount);
    mainMenuScroll = mainMenuIndex;
  }
  if (input::consumeJoystickPress()) {
    systemState.menuMode = MenuMode::Status;
    setUiState(UiState::StatusScreen);
    showInfo("Main menu closed", 1500);
    return;
  }
  bool select = input::consumeEncoderClick();
  if (!select) {
    return;
  }
  switch (mainMenuIndex) {
    case 0:
      systemState.menuMode = MenuMode::Status;
      setUiState(UiState::StatusDetails);
      break;
    case 1:
      enterPolarAlignMenu();
      break;
    case 2:
      if (!systemState.polarAligned) {
        showInfo("Align first");
      } else {
        if (!tracking.active) {
          if (startTrackingCurrentOrientation()) {
            showInfo("Tracking on");
          } else {
            showInfo("Track failed");
          }
        } else {
          tracking.userAdjusting = false;
          motion::setTrackingEnabled(true);
          systemState.trackingActive = true;
          showInfo("Tracking on");
        }
      }
      break;
    case 3:
      stopTracking();
      showInfo("Tracking off");
      break;
    case 4:
      if (catalog::size() == 0 || catalog::typeGroupCount() == 0) {
        showInfo("Catalog missing");
      } else {
        size_t typeCount = catalog::typeGroupCount();
        int targetType = systemState.selectedCatalogTypeIndex;
        if (targetType < 0 || targetType >= static_cast<int>(typeCount)) {
          targetType = 0;
        }

        if (systemState.selectedCatalogIndex >= 0 &&
            systemState.selectedCatalogIndex < static_cast<int>(catalog::size())) {
          int group = catalog::findTypeGroupForObject(
              static_cast<size_t>(systemState.selectedCatalogIndex));
          if (group >= 0) {
            targetType = group;
            catalogIndex = systemState.selectedCatalogIndex;
            int local = catalog::findTypeLocalIndex(static_cast<size_t>(group),
                                                   static_cast<size_t>(systemState.selectedCatalogIndex));
            if (local >= 0) {
              catalogTypeObjectIndex = local;
            } else {
              catalogTypeObjectIndex = 0;
            }
          }
        }

        catalogTypeIndex = targetType;
        catalogItemScroll = 0;
        catalogDetailMenuIndex = 0;

        CatalogTypeSummary summary{};
        if (!catalog::getTypeSummary(static_cast<size_t>(catalogTypeIndex), summary) ||
            summary.objectCount == 0) {
          catalogTypeObjectIndex = 0;
        } else {
          if (catalogTypeObjectIndex < 0) catalogTypeObjectIndex = 0;
          if (catalogTypeObjectIndex >= static_cast<int>(summary.objectCount)) {
            catalogTypeObjectIndex = static_cast<int>(summary.objectCount) - 1;
          }
          size_t globalIndex = 0;
          if (!catalog::getTypeObjectIndex(static_cast<size_t>(catalogTypeIndex),
                                           static_cast<size_t>(catalogTypeObjectIndex), globalIndex)) {
            catalogTypeObjectIndex = 0;
            if (catalog::getTypeObjectIndex(static_cast<size_t>(catalogTypeIndex), 0, globalIndex)) {
              catalogIndex = static_cast<int>(globalIndex);
            }
          } else {
            catalogIndex = static_cast<int>(globalIndex);
          }
          int visibleRows = computeVisibleRows(20, kLineHeight);
          ensureSelectionVisible(catalogItemScroll, catalogTypeObjectIndex, visibleRows, summary.objectCount);
        }

        systemState.menuMode = MenuMode::Catalog;
        setUiState(UiState::CatalogTypeBrowser);
      }
      break;
    case 5:
      startGotoToSelected();
      break;
    case 6:
      enterGotoCoordinateEntry();
      break;
    case 7:
      startParkPosition();
      break;
    case 8:
      enterSetupMenu();
      break;
    default:
      break;
  }
}

void handlePolarAlignMenuInput(int delta) {
  if (delta != 0) {
    polarAlignMenuIndex += delta;
    if (polarAlignMenuIndex < 0) polarAlignMenuIndex = 0;
    if (polarAlignMenuIndex >= static_cast<int>(kPolarAlignMenuCount)) {
      polarAlignMenuIndex = static_cast<int>(kPolarAlignMenuCount) - 1;
    }
  }
  int visible = computeVisibleRows(32, 0);
  ensureSelectionVisible(polarAlignMenuScroll, polarAlignMenuIndex, visible, kPolarAlignMenuCount);
  if (input::consumeJoystickPress()) {
    systemState.menuMode = MenuMode::Status;
    setUiState(UiState::MainMenu);
    return;
  }
  if (!input::consumeEncoderClick()) {
    return;
  }
  switch (polarAlignMenuIndex) {
    case kPolarAlignMenuLockIndex:
      startPolarAlignment();
      break;
    case kPolarAlignMenuRefineIndex:
      if (!systemState.polarAligned || !orientationKnown) {
        showInfo("Lock Polaris first");
      } else {
        startLockingStarFollowup(true);
      }
      break;
    case kPolarAlignMenuClearIndex:
      orientationModel.reset();
      storage::clearOrientationModel();
      showInfo("Refinements cleared");
      break;
    case kPolarAlignMenuBackIndex:
      systemState.menuMode = MenuMode::Status;
      setUiState(UiState::MainMenu);
      break;
  }
}

void handleSetupMenuInput(int delta) {
  if (delta != 0) {
    setupMenuIndex += delta;
    while (setupMenuIndex < 0) setupMenuIndex += static_cast<int>(kSetupMenuCount);
    while (setupMenuIndex >= static_cast<int>(kSetupMenuCount))
      setupMenuIndex -= static_cast<int>(kSetupMenuCount);
    setupMenuScroll = setupMenuIndex;
  }
  if (input::consumeJoystickPress()) {
    setUiState(UiState::MainMenu);
    showInfo("Setup closed", 1500);
    return;
  }
  bool select = input::consumeEncoderClick();
  if (!select) {
    return;
  }
  switch (setupMenuIndex) {
    case kSetupMenuRtcIndex:
      enterRtcEditor();
      break;
    case kSetupMenuLocationIndex:
      enterLocationSetup();
      break;
    case kSetupMenuJoystickIndex:
      startJoystickCalibrationFlow();
      break;
    case kSetupMenuAxisOrientationIndex:
      enterAxisOrientationSetup();
      break;
    case kSetupMenuAxisCalIndex:
      resetAxisCalibrationState();
      showInfo("Set Az 0");
      break;
    case kSetupMenuGotoSpeedIndex:
      enterGotoSpeedSetup();
      break;
    case kSetupMenuPanSpeedIndex:
      enterPanningSpeedSetup();
      break;
    case kSetupMenuBacklashIndex:
      startBacklashCalibration();
      break;
    case kSetupMenuWifiIndex: {
      if (!wifi_ota::credentialsConfigured()) {
        showInfo("WiFi creds missing", 2000);
        break;
      }
      bool enable = !wifi_ota::isEnabled();
      if (enable && stellarium_link::accessPointActive()) {
        stellarium_link::disableAccessPoint();
      }
      wifi_ota::setEnabled(enable);
      String error;
      if (!comm::call("SET_WIFI_ENABLED", {enable ? "1" : "0"}, nullptr, &error)) {
        String message = "Main WiFi: ";
        message += error.isEmpty() ? "failed" : error;
        showInfo(message, 2000);
      } else if (enable) {
        showInfo(String("WiFi: ") + wifi_ota::ssid(), 2500);
      } else {
        showInfo("WiFi disabled", 1500);
      }
      break;
    }
    case kSetupMenuWifiApIndex: {
      bool enable = !stellarium_link::accessPointActive();
      if (enable) {
        if (stellarium_link::enableAccessPoint()) {
          showInfo(String("AP: ") + stellarium_link::accessPointSsid(), 2500);
        } else {
          showInfo("AP failed", 2000);
        }
      } else {
        stellarium_link::disableAccessPoint();
        showInfo("AP disabled", 1500);
      }
      break;
    }
    case kSetupMenuStellariumIndex: {
      if (stellarium_link::clientConnected()) {
        stellarium_link::forceDisconnectClient();
      } else {
        showInfo("No Stellarium client", 2000);
      }
      break;
    }
    case kSetupMenuBackIndex:
      setUiState(UiState::MainMenu);
      break;
    default:
      break;
  }
}

void handleRtcInput(int delta) {
  if (delta != 0) {
    if (rtcEdit.fieldIndex < kRtcFieldCount - 1) {
      switch (rtcEdit.fieldIndex) {
        case 0:
          rtcEdit.year = std::clamp(rtcEdit.year + delta, 2020, 2100);
          break;
        case 1:
          rtcEdit.month += delta;
          if (rtcEdit.month < 1) rtcEdit.month = 12;
          if (rtcEdit.month > 12) rtcEdit.month = 1;
          break;
        case 2:
          rtcEdit.day += delta;
          if (rtcEdit.day < 1) rtcEdit.day = 31;
          if (rtcEdit.day > 31) rtcEdit.day = 1;
          break;
        case 3:
          rtcEdit.hour = (rtcEdit.hour + delta + 24) % 24;
          break;
        case 4:
          rtcEdit.minute = (rtcEdit.minute + delta + 60) % 60;
          break;
        case 5:
          rtcEdit.second = (rtcEdit.second + delta + 60) % 60;
          break;
        case 6: {
          int mode = static_cast<int>(rtcEdit.dstMode);
          mode += delta;
          while (mode < 0) mode += 3;
          while (mode >= 3) mode -= 3;
          rtcEdit.dstMode = static_cast<DstMode>(mode);
          break;
        }
      }
    } else {
      int step = (delta > 0) ? 1 : -1;
      rtcEdit.actionIndex = (rtcEdit.actionIndex + step + 2) % 2;
    }
  }
  if (input::consumeJoystickPress()) {
    setUiState(UiState::SetupMenu);
    showInfo("RTC canceled");
    return;
  }
  if (input::consumeEncoderClick()) {
    if (rtcEdit.fieldIndex < kRtcFieldCount - 1) {
      ++rtcEdit.fieldIndex;
      if (rtcEdit.fieldIndex == kRtcFieldCount - 1) {
        rtcEdit.actionIndex = 0;
      }
      return;
    }
    if (rtcEdit.actionIndex == 0) {
      applyRtcEdit();
    } else {
      setUiState(UiState::SetupMenu);
      showInfo("RTC unchanged");
    }
  }
}

void handleCatalogTypeInput(int delta) {
  size_t typeCount = catalog::typeGroupCount();
  if (typeCount == 0) {
    if (input::consumeEncoderClick() || input::consumeJoystickPress()) {
      setUiState(UiState::MainMenu);
    }
    return;
  }

  int total = static_cast<int>(typeCount);
  if (delta != 0) {
    catalogTypeIndex += delta;
    while (catalogTypeIndex < 0) catalogTypeIndex += total;
    while (catalogTypeIndex >= total) catalogTypeIndex -= total;
    catalogTypeObjectIndex = 0;
    catalogItemScroll = 0;
  }

  if (input::consumeJoystickPress()) {
    systemState.menuMode = MenuMode::Status;
    setUiState(UiState::MainMenu);
    return;
  }

  if (input::consumeEncoderClick()) {
    CatalogTypeSummary summary{};
    if (!catalog::getTypeSummary(static_cast<size_t>(catalogTypeIndex), summary) ||
        summary.objectCount == 0) {
      showInfo("Empty type");
      return;
    }
    if (catalogTypeObjectIndex >= static_cast<int>(summary.objectCount)) {
      catalogTypeObjectIndex = 0;
    }
    size_t globalIndex = 0;
    if (!catalog::getTypeObjectIndex(static_cast<size_t>(catalogTypeIndex),
                                     static_cast<size_t>(catalogTypeObjectIndex), globalIndex)) {
      showInfo("Invalid entry");
      return;
    }
    catalogIndex = static_cast<int>(globalIndex);
    catalogDetailMenuIndex = 0;
    systemState.selectedCatalogTypeIndex = catalogTypeIndex;
    setUiState(UiState::CatalogItemList);
  }
}

void handleCatalogItemListInput(int delta) {
  if (catalog::size() == 0 || catalog::typeGroupCount() == 0) {
    bool exit = input::consumeEncoderClick();
    if (!exit) {
      exit = input::consumeJoystickPress();
    }
    if (exit) {
      setUiState(UiState::CatalogTypeBrowser);
    }
    return;
  }

  CatalogTypeSummary summary{};
  if (!catalog::getTypeSummary(static_cast<size_t>(catalogTypeIndex), summary) ||
      summary.objectCount == 0) {
    showInfo("Empty type");
    setUiState(UiState::CatalogTypeBrowser);
    return;
  }

  int total = static_cast<int>(summary.objectCount);
  int visibleRows = computeVisibleRows(20, kLineHeight);
  if (delta != 0) {
    catalogTypeObjectIndex += delta;
    while (catalogTypeObjectIndex < 0) catalogTypeObjectIndex += total;
    while (catalogTypeObjectIndex >= total) catalogTypeObjectIndex -= total;
  }

  ensureSelectionVisible(catalogItemScroll, catalogTypeObjectIndex, visibleRows, summary.objectCount);

  size_t globalIndex = 0;
  if (!catalog::getTypeObjectIndex(static_cast<size_t>(catalogTypeIndex),
                                   static_cast<size_t>(catalogTypeObjectIndex), globalIndex)) {
    showInfo("Invalid entry");
    setUiState(UiState::CatalogTypeBrowser);
    return;
  }
  catalogIndex = static_cast<int>(globalIndex);

  if (input::consumeEncoderClick()) {
    catalogDetailMenuIndex = 0;
    catalogDetailSelectingAction = false;
    systemState.selectedCatalogTypeIndex = catalogTypeIndex;
    setUiState(UiState::CatalogItemDetail);
    return;
  }

  if (input::consumeJoystickPress()) {
    setUiState(UiState::CatalogTypeBrowser);
  }
}

void handleCatalogItemDetailInput(int delta) {
  if (catalog::size() == 0 || catalog::typeGroupCount() == 0) {
    catalogDetailSelectingAction = false;
    setUiState(UiState::CatalogTypeBrowser);
    return;
  }

  CatalogTypeSummary summary{};
  if (!catalog::getTypeSummary(static_cast<size_t>(catalogTypeIndex), summary) ||
      summary.objectCount == 0) {
    showInfo("Empty type");
    catalogDetailSelectingAction = false;
    setUiState(UiState::CatalogTypeBrowser);
    return;
  }

  int total = static_cast<int>(summary.objectCount);
  if (catalogTypeObjectIndex < 0 || catalogTypeObjectIndex >= total) {
    catalogTypeObjectIndex = ((catalogTypeObjectIndex % total) + total) % total;
  }

  if (delta != 0) {
    if (catalogDetailSelectingAction) {
      catalogDetailMenuIndex += delta;
      while (catalogDetailMenuIndex < 0) catalogDetailMenuIndex += kCatalogDetailMenuCount;
      while (catalogDetailMenuIndex >= kCatalogDetailMenuCount)
        catalogDetailMenuIndex -= kCatalogDetailMenuCount;
    } else {
      catalogTypeObjectIndex += delta;
      while (catalogTypeObjectIndex < 0) catalogTypeObjectIndex += total;
      while (catalogTypeObjectIndex >= total) catalogTypeObjectIndex -= total;
    }
  }

  size_t globalIndex = 0;
  if (!catalog::getTypeObjectIndex(static_cast<size_t>(catalogTypeIndex),
                                   static_cast<size_t>(catalogTypeObjectIndex), globalIndex)) {
    showInfo("Invalid entry");
    catalogDetailSelectingAction = false;
    setUiState(UiState::CatalogTypeBrowser);
    return;
  }

  catalogIndex = static_cast<int>(globalIndex);
  if (catalogDetailMenuIndex < 0 || catalogDetailMenuIndex >= kCatalogDetailMenuCount) {
    catalogDetailMenuIndex = 0;
  }

  if (input::consumeEncoderClick()) {
    if (!catalogDetailSelectingAction) {
      catalogDetailSelectingAction = true;
      catalogDetailMenuIndex = 0;
      return;
    }

    if (catalogDetailMenuIndex == 0) {
      const CatalogObject* object = catalog::get(globalIndex);
      if (object && startGotoToObject(*object, catalogIndex)) {
        selectedObjectName = sanitizeForDisplay(object->name);
        gotoTargetName = sanitizeForDisplay(object->name);
      }
      catalogDetailSelectingAction = false;
    } else {
      catalogDetailSelectingAction = false;
      setUiState(UiState::CatalogItemList);
      return;
    }
  }

  if (input::consumeJoystickPress()) {
    if (catalogDetailSelectingAction) {
      catalogDetailSelectingAction = false;
    } else {
      setUiState(UiState::CatalogItemList);
    }
  }
}

void handleStartupLockPromptInput(int delta) {
  if (!startupPromptActive) {
    setUiState(UiState::StatusScreen);
    return;
  }

  if (delta != 0) {
    startupPromptIndex += delta;
    while (startupPromptIndex < 0) startupPromptIndex += static_cast<int>(kStartupPromptCount);
    while (startupPromptIndex >= static_cast<int>(kStartupPromptCount))
      startupPromptIndex -= static_cast<int>(kStartupPromptCount);
  }

  bool confirm = input::consumeEncoderClick();
  if (!confirm) {
    confirm = input::consumeJoystickPress();
  }
  if (!confirm) {
    return;
  }

  startupPromptActive = false;

  switch (startupPromptIndex) {
    case 0:  // Use saved lock
      systemState.polarAligned = true;
      storage::setPolarAligned(true);
      setOrientationKnown(true);
      systemState.menuMode = MenuMode::Status;
      setUiState(UiState::StatusScreen);
      showInfo("Using saved lock", 2000);
      break;
    case 1:  // New Polaris lock
      startPolarAlignment();
      break;
    default:  // Discard lock
      systemState.polarAligned = false;
      storage::setPolarAligned(false);
      setOrientationKnown(false);
      systemState.menuMode = MenuMode::Status;
      setUiState(UiState::StatusScreen);
      showInfo("Lock discarded", 2000);
      break;
  }
}

void handleLockingStarMenuInput(int delta) {
  if (!lockingStarFlowActive) {
    setUiState(UiState::StatusScreen);
    return;
  }
  if (lockingStarOptionCount > 0 && delta != 0) {
    lockingStarSelectionIndex += delta;
    while (lockingStarSelectionIndex < 0) {
      lockingStarSelectionIndex += static_cast<int>(lockingStarOptionCount);
    }
    while (lockingStarSelectionIndex >= static_cast<int>(lockingStarOptionCount)) {
      lockingStarSelectionIndex -= static_cast<int>(lockingStarOptionCount);
    }
  }

  if (input::consumeEncoderClick()) {
    if (lockingStarOptionCount == 0) {
      showInfo("Refine skipped", 2000);
      finishLockingStarFlow();
      return;
    }
    int index = lockingStarSelectionIndex;
    if (index < 0 || index >= static_cast<int>(lockingStarOptionCount)) {
      index = 0;
    }
    if (startLockingStarGoto(lockingStarOptions[static_cast<size_t>(index)])) {
      return;
    }
  }

  if (input::consumeJoystickPress()) {
    showInfo("Refine skipped", 2000);
    finishLockingStarFlow();
  }
}

void handleLockingStarRefineInput() {
  if (!lockingStarFlowActive) {
    setUiState(UiState::StatusScreen);
    return;
  }
  if (input::consumeEncoderClick()) {
    finalizeLockingStarRefinement();
    return;
  }
  if (input::consumeJoystickPress()) {
    showInfo("Lock not saved", 2000);
    finishLockingStarFlow();
  }
}

void handlePolarAlignInput() {
  bool select = input::consumeEncoderClick();
  if (select) {
    completePolarAlignment();
  }
  if (input::consumeJoystickPress()) {
    systemState.menuMode = MenuMode::Status;
    setUiState(UiState::StatusScreen);
    showInfo("Align aborted");
  }
}

}  // namespace

void applyNetworkTime(time_t utcEpoch) {
  if (rtcAvailable) {
    MutexLock lock(i2cMutex);
    if (lock.locked()) {
      rtc.adjust(DateTime(utcEpoch));
    }
  }
  storage::setRtcEpoch(static_cast<uint32_t>(utcEpoch));
}

void stopTracking() {
  tracking.active = false;
  tracking.userAdjusting = false;
  systemState.trackingActive = false;
  motion::setTrackingEnabled(false);
  motion::setTrackingRates(0.0, 0.0);
}

void applyOrientationState(bool known) {
  orientationKnown = known;
  if (!known) {
    stopTracking();
    motion::setAltitudeLimitsEnabled(false);
    gotoRuntime.active = false;
    systemState.gotoActive = false;
    motion::clearGotoRates();
    motion::setStepCount(Axis::Az, 0);
    motion::setStepCount(Axis::Alt, 0);
  } else {
    motion::setAltitudeLimitsEnabled(true);
  }
}

void setOrientationKnown(bool known) { applyOrientationState(known); }

void init() {
  if (i2cMutex == nullptr) {
    i2cMutex = xSemaphoreCreateRecursiveMutexStatic(&i2cMutexBuffer);
  }

  Wire.begin(config::SDA_PIN, config::SCL_PIN);

  orientationModel.loadFromConfig(storage::getConfig());

  auto initPeripherals = [&]() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      // OLED init failure will be reported via on-screen message; avoid serial
      // output because the primary UART is reserved for the inter-board link.
    }
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.display();

    rtcAvailable = rtc.begin();
    if (!rtcAvailable) {
      delay(25);
      rtcAvailable = rtc.begin();
    }
  };

  {
    MutexLock lock(i2cMutex);
    (void)lock;
    initPeripherals();
  }

  if (!rtcAvailable) {
    showInfo("RTC missing", 2000);
  }
}

void showBootMessage() {
  MutexLock lock(i2cMutex);
  if (!lock.locked()) {
    return;
  }
  for (size_t i = 0; i < kBootStarCount; ++i) {
    resetBootStar(i);
  }
  bootAnimationActive = true;
  bootAnimationStopRequested = false;
  bootAnimationStartMs = millis();
  bootAnimationMinEndMs = bootAnimationStartMs + kBootAnimationMinDurationMs;
  drawBootAnimation(bootAnimationStartMs);
  display.display();
}

void stopBootAnimation() {
  if (!bootAnimationActive) {
    return;
  }
  uint32_t now = millis();
  if (now >= bootAnimationMinEndMs) {
    bootAnimationActive = false;
    bootAnimationStopRequested = false;
  } else {
    bootAnimationStopRequested = true;
  }
}

void showCalibrationStart() {
  MutexLock lock(i2cMutex);
  if (!lock.locked()) {
    return;
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Calibrating joystick");
  display.display();
}

void showCalibrationResult(int centerX, int centerY) {
  {
    MutexLock lock(i2cMutex);
    if (!lock.locked()) {
      return;
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Calibration done");
    display.setCursor(0, 16);
    display.printf("CX=%d", centerX);
    display.setCursor(0, 24);
    display.printf("CY=%d", centerY);
    display.display();
  }
  delay(1000);
}

void showReady() {
  bool hasActiveMessage = false;
  portENTER_CRITICAL(&displayMux);
  if (!infoMessage.isEmpty() && millis() <= infoUntil) {
    hasActiveMessage = true;
  }
  portEXIT_CRITICAL(&displayMux);
  if (!hasActiveMessage) {
    showInfo("NERDSTAR ready", 1500);
  }
}

void prepareStartupLockPrompt(bool hasSavedLock) {
  startupPromptActive = hasSavedLock;
  startupPromptIndex = 0;
  systemState.menuMode = MenuMode::Status;
  if (hasSavedLock) {
    setUiState(UiState::StartupLockPrompt);
  } else {
    setUiState(UiState::StatusScreen);
  }
}

void startTask() {
  xTaskCreatePinnedToCore(displayTask, "display", 4096, nullptr, 1, nullptr, 0);
}

void handleStatusScreenInput() {
  if (input::consumeEncoderClick()) {
    mainMenuIndex = 0;
    mainMenuScroll = 0;
    setUiState(UiState::MainMenu);
  }
}

void handleStatusDetailsInput() {
  if (input::consumeJoystickPress()) {
    systemState.menuMode = MenuMode::Status;
    setUiState(UiState::StatusScreen);
    return;
  }
  if (input::consumeEncoderClick()) {
    mainMenuIndex = 0;
    mainMenuScroll = 0;
    setUiState(UiState::MainMenu);
  }
}

void handleInput() {
  input::update();
  int delta = input::consumeEncoderDelta();

  switch (uiState) {
    case UiState::StatusScreen:
      handleStatusScreenInput();
      break;
    case UiState::StatusDetails:
      handleStatusDetailsInput();
      break;
    case UiState::StartupLockPrompt:
      handleStartupLockPromptInput(delta);
      break;
    case UiState::MainMenu:
      handleMainMenuInput(delta);
      break;
    case UiState::PolarAlignMenu:
      handlePolarAlignMenuInput(delta);
      break;
    case UiState::PolarAlign:
      handlePolarAlignInput();
      break;
    case UiState::LockingStarMenu:
      handleLockingStarMenuInput(delta);
      break;
    case UiState::LockingStarRefine:
      handleLockingStarRefineInput();
      break;
    case UiState::SetupMenu:
      handleSetupMenuInput(delta);
      break;
    case UiState::SetRtc:
      handleRtcInput(delta);
      break;
    case UiState::LocationSetup:
      handleLocationInput(delta);
      break;
    case UiState::AxisOrientation:
      handleAxisOrientationInput(delta);
      break;
    case UiState::CatalogTypeBrowser:
      handleCatalogTypeInput(delta);
      break;
    case UiState::CatalogItemList:
      handleCatalogItemListInput(delta);
      break;
    case UiState::CatalogItemDetail:
      handleCatalogItemDetailInput(delta);
      break;
    case UiState::AxisCalibration: {
      if (input::consumeJoystickPress()) {
        setUiState(UiState::SetupMenu);
        break;
      }
      bool select = input::consumeEncoderClick();
      if (select) {
        handleAxisCalibrationClick();
      }
      break;
    }
    case UiState::GotoSpeed:
    case UiState::PanningSpeed:
      handleSpeedProfileInput(delta);
      break;
    case UiState::GotoCoordinates:
      handleGotoCoordinateInput(delta);
      break;
    case UiState::BacklashCalibration:
      handleBacklashCalibrationInput();
      break;
  }
}

void showInfo(const String& message, uint32_t durationMs) {
  uint32_t until = millis() + durationMs;
  portENTER_CRITICAL(&displayMux);
  infoMessage = sanitizeForDisplay(message);
  infoUntil = until;
  portEXIT_CRITICAL(&displayMux);
}

void completePolarAlignment() {
  systemState.menuMode = MenuMode::Status;
  systemState.polarAligned = true;
  systemState.trackingActive = false;
  systemState.gotoActive = false;
  stopTracking();
  double azDeg = 0.0;
  double altDeg = 0.0;
  DateTime now = currentDateTime();
  if (raDecToAltAz(now, config::POLARIS_RA_HOURS, config::POLARIS_DEC_DEGREES, azDeg, altDeg)) {
    motion::setStepCount(Axis::Az, motion::azDegreesToSteps(azDeg));
    motion::setStepCount(Axis::Alt, motion::altDegreesToSteps(altDeg));
  }
  storage::setPolarAligned(true);
  applyOrientationState(true);
  setUiState(UiState::StatusScreen);
  bool trackingStarted = startTrackingCurrentOrientation();
  showInfo(trackingStarted ? "Tracking Polaris" : "Polaris locked");
  startLockingStarFollowup();
}

void startPolarAlignment() {
  systemState.menuMode = MenuMode::PolarAlign;
  systemState.polarAligned = false;
  systemState.trackingActive = false;
  systemState.gotoActive = false;
  applyOrientationState(false);
  storage::setPolarAligned(false);
  setUiState(UiState::PolarAlign);
  resetLockingStarFlow();
  showInfo("Use joystick", 2000);
}

void update() { updateGoto(); }

}  // namespace display_menu

#endif  // DEVICE_ROLE_HID

