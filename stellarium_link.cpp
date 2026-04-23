#include "stellarium_link.h"

#include <cmath>

#include "config.h"
#include "motion.h"

namespace stellarium_link {
namespace {

String g_usbCommandBuffer;

enum class SlewMode : uint8_t {
  kIdle,
  kGoto,
};

struct Lx200State {
  double targetRaDeg = 0.0;
  double targetDecDeg = 0.0;
  bool hasTargetRa = false;
  bool hasTargetDec = false;
  SlewMode slewMode = SlewMode::kIdle;
  int8_t manualRaDir = 0;
  int8_t manualDecDir = 0;
};

Lx200State g_lx200;

double normalizeDegrees360(double value) {
  double wrapped = fmod(value, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped;
}

double shortestAngularDistanceDegrees(double fromDeg, double toDeg) {
  double diff = normalizeDegrees360(toDeg) - normalizeDegrees360(fromDeg);
  if (diff > 180.0) {
    diff -= 360.0;
  } else if (diff < -180.0) {
    diff += 360.0;
  }
  return diff;
}

String formatHMS(uint8_t h, uint8_t m, uint8_t s) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", h, m, s);
  return String(buffer);
}

String formatRa(double raDeg) {
  raDeg = normalizeDegrees360(raDeg);
  double totalHours = raDeg / 15.0;
  int32_t totalSeconds = static_cast<int32_t>(lround(totalHours * 3600.0));
  totalSeconds %= (24 * 3600);
  if (totalSeconds < 0) {
    totalSeconds += 24 * 3600;
  }
  uint8_t hour = static_cast<uint8_t>(totalSeconds / 3600);
  uint8_t minute = static_cast<uint8_t>((totalSeconds % 3600) / 60);
  uint8_t second = static_cast<uint8_t>(totalSeconds % 60);
  return formatHMS(hour, minute, second);
}

String formatDMS(int16_t d, uint8_t m, uint8_t s) {
  char sign = d < 0 ? '-' : '+';
  int16_t absD = d < 0 ? -d : d;
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%c%02d*%02u:%02u", sign, absD, m, s);
  return String(buffer);
}

// LX200 clients expect declination strictly within [-90, +90] degrees.
// The motion layer can return values outside that range (e.g. after
// wrap-around on an uncalibrated axis), so we clamp here before emitting.
String formatDec(double decDeg) {
  if (decDeg > 90.0) {
    decDeg = 90.0;
  } else if (decDeg < -90.0) {
    decDeg = -90.0;
  }

  int32_t totalArcSeconds = static_cast<int32_t>(lround(decDeg * 3600.0));
  int sign = totalArcSeconds < 0 ? -1 : 1;
  int32_t absArcSeconds = std::abs(totalArcSeconds);
  int16_t degrees = static_cast<int16_t>(absArcSeconds / 3600);
  uint8_t minutes = static_cast<uint8_t>((absArcSeconds % 3600) / 60);
  uint8_t seconds = static_cast<uint8_t>(absArcSeconds % 60);
  return formatDMS(static_cast<int16_t>(degrees * sign), minutes, seconds);
}

bool parseUnsignedPart(const String& text, int& outValue) {
  if (text.isEmpty()) {
    return false;
  }
  for (uint16_t i = 0; i < text.length(); ++i) {
    if (!isDigit(text[i])) {
      return false;
    }
  }
  outValue = text.toInt();
  return true;
}

bool parseLx200Ra(const String& text, double& outDegrees) {
  int firstColon = text.indexOf(':');
  int secondColon = text.indexOf(':', firstColon + 1);
  if (firstColon <= 0 || secondColon <= firstColon) {
    return false;
  }

  int hour = 0;
  int minute = 0;
  int second = 0;
  if (!parseUnsignedPart(text.substring(0, firstColon), hour) ||
      !parseUnsignedPart(text.substring(firstColon + 1, secondColon), minute) ||
      !parseUnsignedPart(text.substring(secondColon + 1), second)) {
    return false;
  }

  if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 ||
      second > 59) {
    return false;
  }

  double totalHours = static_cast<double>(hour) +
                      static_cast<double>(minute) / 60.0 +
                      static_cast<double>(second) / 3600.0;
  outDegrees = totalHours * 15.0;
  return true;
}

bool parseLx200Dec(const String& text, double& outDegrees) {
  if (text.length() < 7) {
    return false;
  }

  int sign = 1;
  int offset = 0;
  if (text[0] == '-') {
    sign = -1;
    offset = 1;
  } else if (text[0] == '+') {
    offset = 1;
  }

  int degreeSep = text.indexOf('*', offset);
  if (degreeSep < 0) {
    degreeSep = text.indexOf(':', offset);
  }
  int minuteSep = text.indexOf(':', degreeSep + 1);
  if (degreeSep <= offset || minuteSep <= degreeSep) {
    return false;
  }

  int degrees = 0;
  int minutes = 0;
  int seconds = 0;
  if (!parseUnsignedPart(text.substring(offset, degreeSep), degrees) ||
      !parseUnsignedPart(text.substring(degreeSep + 1, minuteSep), minutes) ||
      !parseUnsignedPart(text.substring(minuteSep + 1), seconds)) {
    return false;
  }

  if (degrees > 180 || minutes > 59 || seconds > 59) {
    return false;
  }

  double value = static_cast<double>(degrees) +
                 static_cast<double>(minutes) / 60.0 +
                 static_cast<double>(seconds) / 3600.0;
  outDegrees = value * static_cast<double>(sign);
  return true;
}

void writeReplyToUsb(const String& payload) {
  if (!Serial) {
    return;
  }
  Serial.print(payload);
  Serial.print('#');
}

void resetSlewState() {
  g_lx200.slewMode = SlewMode::kIdle;
  motion::clearGotoRates();
}

void applyManualMotion() {
  int64_t raStepsPerDegree = llabs(motion::raDegreesToSteps(1.0) - motion::raDegreesToSteps(0.0));
  int64_t decStepsPerDegree =
      llabs(motion::decDegreesToSteps(1.0) - motion::decDegreesToSteps(0.0));

  double raManualSps = static_cast<double>(g_lx200.manualRaDir) *
                       config::LX200_MANUAL_SPEED_DEG_PER_SEC *
                       static_cast<double>(raStepsPerDegree);
  double decManualSps = static_cast<double>(g_lx200.manualDecDir) *
                        config::LX200_MANUAL_SPEED_DEG_PER_SEC *
                        static_cast<double>(decStepsPerDegree);

  motion::setManualStepsPerSecond(Axis::Ra, raManualSps);
  motion::setManualStepsPerSecond(Axis::Dec, decManualSps);
}

void updateGotoSlew() {
  if (g_lx200.slewMode != SlewMode::kGoto || !g_lx200.hasTargetRa ||
      !g_lx200.hasTargetDec) {
    return;
  }

  double currentRa = motion::stepsToRaDegrees(motion::getStepCount(Axis::Ra));
  double currentDec = motion::stepsToDecDegrees(motion::getStepCount(Axis::Dec));

  double raError = shortestAngularDistanceDegrees(currentRa, g_lx200.targetRaDeg);
  double decError = g_lx200.targetDecDeg - currentDec;

  if (fabs(raError) <= config::LX200_GOTO_TOLERANCE_DEG &&
      fabs(decError) <= config::LX200_GOTO_TOLERANCE_DEG) {
    resetSlewState();
    return;
  }

  int64_t raStepsPerDegree = llabs(motion::raDegreesToSteps(1.0) - motion::raDegreesToSteps(0.0));
  int64_t decStepsPerDegree =
      llabs(motion::decDegreesToSteps(1.0) - motion::decDegreesToSteps(0.0));

  double raSps = (raError > 0.0 ? 1.0 : -1.0) *
                 config::LX200_GOTO_SPEED_DEG_PER_SEC *
                 static_cast<double>(raStepsPerDegree);
  double decSps = (decError > 0.0 ? 1.0 : -1.0) *
                  config::LX200_GOTO_SPEED_DEG_PER_SEC *
                  static_cast<double>(decStepsPerDegree);

  if (fabs(raError) <= config::LX200_GOTO_TOLERANCE_DEG) {
    raSps = 0.0;
  }
  if (fabs(decError) <= config::LX200_GOTO_TOLERANCE_DEG) {
    decSps = 0.0;
  }

  motion::setGotoStepsPerSecond(Axis::Ra, raSps);
  motion::setGotoStepsPerSecond(Axis::Dec, decSps);
}

template <typename ReplyFn>
void handleLx200Command(const String& cmd, ReplyFn&& reply) {
  if (cmd == ":Q") {
    g_lx200.manualRaDir = 0;
    g_lx200.manualDecDir = 0;
    applyManualMotion();
    resetSlewState();
    reply("1");
    return;
  }

  if (cmd == ":Qn" || cmd == ":Qs") {
    g_lx200.manualDecDir = 0;
    applyManualMotion();
    reply("1");
    return;
  }
  if (cmd == ":Qe" || cmd == ":Qw") {
    g_lx200.manualRaDir = 0;
    applyManualMotion();
    reply("1");
    return;
  }

  if (cmd == ":Mn") {
    g_lx200.manualDecDir = 1;
    resetSlewState();
    applyManualMotion();
    reply("1");
    return;
  }
  if (cmd == ":Ms") {
    g_lx200.manualDecDir = -1;
    resetSlewState();
    applyManualMotion();
    reply("1");
    return;
  }
  if (cmd == ":Me") {
    g_lx200.manualRaDir = 1;
    resetSlewState();
    applyManualMotion();
    reply("1");
    return;
  }
  if (cmd == ":Mw") {
    g_lx200.manualRaDir = -1;
    resetSlewState();
    applyManualMotion();
    reply("1");
    return;
  }

  if (cmd == ":GR") {
    double ra = motion::stepsToRaDegrees(motion::getStepCount(Axis::Ra));
    reply(formatRa(ra));
    return;
  }

  if (cmd == ":GD") {
    double dec = motion::stepsToDecDegrees(motion::getStepCount(Axis::Dec));
    reply(formatDec(dec));
    return;
  }

  // Product / firmware identification. INDI's LX200 drivers issue these
  // during handshake and refuse to progress without a sensible response.
  if (cmd == ":GVP") {
    reply("NERDSTAR-eQ");
    return;
  }
  if (cmd == ":GVN") {
    reply(config::LX200_FIRMWARE_VERSION);
    return;
  }
  if (cmd == ":GVD") {
    reply(__DATE__);
    return;
  }
  if (cmd == ":GVT") {
    reply(__TIME__);
    return;
  }
  if (cmd == ":GVF") {
    reply(String("NERDSTAR-eQ ") + config::LX200_FIRMWARE_VERSION +
          " built " + __DATE__);
    return;
  }

  if (cmd.startsWith(":Sr")) {
    double raTarget = 0.0;
    if (!parseLx200Ra(cmd.substring(3), raTarget)) {
      reply("0");
      return;
    }
    g_lx200.targetRaDeg = normalizeDegrees360(raTarget);
    g_lx200.hasTargetRa = true;
    reply("1");
    return;
  }

  if (cmd.startsWith(":Sd")) {
    double decTarget = 0.0;
    if (!parseLx200Dec(cmd.substring(3), decTarget)) {
      reply("0");
      return;
    }
    g_lx200.targetDecDeg = decTarget;
    g_lx200.hasTargetDec = true;
    reply("1");
    return;
  }

  if (cmd == ":MS") {
    if (!g_lx200.hasTargetRa || !g_lx200.hasTargetDec) {
      reply("1<no target>");
      return;
    }
    g_lx200.manualRaDir = 0;
    g_lx200.manualDecDir = 0;
    applyManualMotion();
    g_lx200.slewMode = SlewMode::kGoto;
    updateGotoSlew();
    reply("0");
    return;
  }

  // Distance bars. Return empty (idle) or non-empty (slewing) body.
  if (cmd == ":D") {
    reply(g_lx200.slewMode == SlewMode::kGoto ? "|" : "");
    return;
  }

  // Site / location / time commands are acknowledged so that INDI-style
  // hosts can progress their handshake, even though the firmware itself
  // does not yet use geographic coordinates.
  if (cmd.startsWith(":SC") || cmd.startsWith(":SL") ||
      cmd.startsWith(":SG") || cmd.startsWith(":Sg") ||
      cmd.startsWith(":St")) {
    reply("1");
    return;
  }

  // Slew-rate selection, tracking-rate selection and precision toggle are
  // accepted silently - the LX200 spec does not require a response and the
  // firmware does not currently map them to mount behaviour.
  if (cmd == ":RG" || cmd == ":RC" || cmd == ":RM" || cmd == ":RS" ||
      cmd.startsWith(":Rs") || cmd.startsWith(":Rg")) {
    return;
  }
  if (cmd == ":TQ" || cmd == ":TS" || cmd == ":TL" || cmd == ":T+" ||
      cmd == ":T-" || cmd.startsWith(":TM")) {
    return;
  }
  if (cmd == ":U") {
    return;
  }

  // Park / home. Acknowledge as "done" - stopAll() halts motion.
  if (cmd == ":hP" || cmd == ":hC" || cmd == ":hF") {
    motion::stopAll();
    reply("1");
    return;
  }

  reply("0");
}

void processUsbClient() {
  while (Serial && Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '#') {
      handleLx200Command(g_usbCommandBuffer,
                         [&](const String& payload) {
                           writeReplyToUsb(payload);
                         });
      g_usbCommandBuffer = "";
    } else if (isPrintable(static_cast<unsigned char>(c))) {
      if (g_usbCommandBuffer.length() < 96) {
        g_usbCommandBuffer += c;
      }
    }
  }
}

}  // namespace

void init() {
  g_usbCommandBuffer = "";
  g_lx200 = Lx200State{};
}

void update() {
  processUsbClient();
  updateGotoSlew();
}

}  // namespace stellarium_link
