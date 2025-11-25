#include "input.h"

#if defined(DEVICE_ROLE_HID)

#include <AiEsp32RotaryEncoder.h>
#include <math.h>
#include <type_traits>
#include <utility>

#include "config.h"
#include "freertos/portmacro.h"

namespace {

static_assert(config::JOYSTICK_X_DIRECTION == 1 || config::JOYSTICK_X_DIRECTION == -1,
              "JOYSTICK_X_DIRECTION must be either 1 or -1");
static_assert(config::JOYSTICK_Y_DIRECTION == 1 || config::JOYSTICK_Y_DIRECTION == -1,
              "JOYSTICK_Y_DIRECTION must be either 1 or -1");

constexpr int kEncoderStepsPerNotch = 2;
constexpr long kEncoderMinValue = -100000;
constexpr long kEncoderMaxValue = 100000;
constexpr uint32_t kAccelerationResetMs = 400;
constexpr int kMaxAcceleratedStep = 6;
constexpr uint32_t kRotaryButtonDebounceMs = 35;

AiEsp32RotaryEncoder rotaryEncoder(config::ROT_A, config::ROT_B, config::ROT_BTN, -1);

portMUX_TYPE rotaryButtonMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool rotaryButtonEdge = false;
bool rotaryButtonPressed = false;
bool rotaryButtonClicked = false;
uint32_t lastRotaryButtonChangeMs = 0;

template <typename...>
using void_t = void;

template <typename T, typename = void>
struct HasSetEncoderStepsPerNotch : std::false_type {};

template <typename T>
struct HasSetEncoderStepsPerNotch<
    T, void_t<decltype(std::declval<T&>().setEncoderStepsPerNotch(0))>>
    : std::true_type {};

template <typename T, typename = void>
struct HasSetStepsPerNotch : std::false_type {};

template <typename T>
struct HasSetStepsPerNotch<
    T, void_t<decltype(std::declval<T&>().setStepsPerNotch(0))>>
    : std::true_type {};

template <typename T, typename = void>
struct HasSetStepsPerClick : std::false_type {};

template <typename T>
struct HasSetStepsPerClick<
    T, void_t<decltype(std::declval<T&>().setStepsPerClick(0))>>
    : std::true_type {};

template <typename T, typename = void>
struct HasLoop : std::false_type {};

template <typename T>
struct HasLoop<T, void_t<decltype(std::declval<T&>().loop())>>
    : std::true_type {};

template <typename T, typename = void>
struct HasTick : std::false_type {};

template <typename T>
struct HasTick<T, void_t<decltype(std::declval<T&>().tick())>>
    : std::true_type {};

template <typename T>
struct EncoderSupportsStepConfiguration
    : std::integral_constant<bool, HasSetEncoderStepsPerNotch<T>::value ||
                                       HasSetStepsPerNotch<T>::value ||
                                       HasSetStepsPerClick<T>::value> {};

template <typename Encoder>
typename std::enable_if<HasSetEncoderStepsPerNotch<Encoder>::value>::type
ConfigureEncoderSteps(Encoder& encoder) {
  encoder.setEncoderStepsPerNotch(kEncoderStepsPerNotch);
}

template <typename Encoder>
typename std::enable_if<!HasSetEncoderStepsPerNotch<Encoder>::value &&
                        HasSetStepsPerNotch<Encoder>::value>::type
ConfigureEncoderSteps(Encoder& encoder) {
  encoder.setStepsPerNotch(kEncoderStepsPerNotch);
}

template <typename Encoder>
typename std::enable_if<!HasSetEncoderStepsPerNotch<Encoder>::value &&
                        !HasSetStepsPerNotch<Encoder>::value &&
                        HasSetStepsPerClick<Encoder>::value>::type
ConfigureEncoderSteps(Encoder& encoder) {
  encoder.setStepsPerClick(kEncoderStepsPerNotch);
}

template <typename Encoder>
typename std::enable_if<!HasSetEncoderStepsPerNotch<Encoder>::value &&
                        !HasSetStepsPerNotch<Encoder>::value &&
                        !HasSetStepsPerClick<Encoder>::value>::type
ConfigureEncoderSteps(Encoder&) {}

template <typename Encoder>
typename std::enable_if<EncoderSupportsStepConfiguration<Encoder>::value>::type
ConfigureEncoderBoundaries(Encoder& encoder) {
  encoder.setBoundaries(kEncoderMinValue, kEncoderMaxValue, true);
}

template <typename Encoder>
typename std::enable_if<!EncoderSupportsStepConfiguration<Encoder>::value>::type
ConfigureEncoderBoundaries(Encoder& encoder) {
  encoder.setBoundaries(kEncoderMinValue * kEncoderStepsPerNotch,
                        kEncoderMaxValue * kEncoderStepsPerNotch, true);
}

template <typename Encoder>
typename std::enable_if<HasLoop<Encoder>::value>::type
ServiceEncoder(Encoder& encoder) {
  encoder.loop();
}

template <typename Encoder>
typename std::enable_if<!HasLoop<Encoder>::value && HasTick<Encoder>::value>::type
ServiceEncoder(Encoder& encoder) {
  encoder.tick();
}

template <typename Encoder>
typename std::enable_if<!HasLoop<Encoder>::value && !HasTick<Encoder>::value>::type
ServiceEncoder(Encoder&) {}

template <typename Encoder>
typename std::enable_if<EncoderSupportsStepConfiguration<Encoder>::value, long>::type
ReadEncoderValue(Encoder& encoder) {
  return encoder.readEncoder();
}

template <typename Encoder>
typename std::enable_if<!EncoderSupportsStepConfiguration<Encoder>::value, long>::type
ReadEncoderValue(Encoder& encoder) {
  long raw = encoder.readEncoder();
  return raw / kEncoderStepsPerNotch;
}

JoystickCalibration currentCalibration{2048, 2048};
bool joystickClick = false;
bool lastJoystickState = false;

long lastEncoderValue = 0;
uint32_t lastEncoderEventMs = 0;
float encoderAccelerationRemainder = 0.0f;

void IRAM_ATTR handleEncoderISR() { rotaryEncoder.readEncoder_ISR(); }

void IRAM_ATTR handleEncoderButtonISR() {
  portENTER_CRITICAL_ISR(&rotaryButtonMux);
  rotaryButtonEdge = true;
  portEXIT_CRITICAL_ISR(&rotaryButtonMux);
}

bool consumeRotaryButtonEdge() {
  portENTER_CRITICAL(&rotaryButtonMux);
  bool edge = rotaryButtonEdge;
  rotaryButtonEdge = false;
  portEXIT_CRITICAL(&rotaryButtonMux);
  return edge;
}

void updateRotaryButton() {
  if (!consumeRotaryButtonEdge()) {
    return;
  }

  uint32_t now = millis();
  if ((now - lastRotaryButtonChangeMs) < kRotaryButtonDebounceMs) {
    return;
  }
  lastRotaryButtonChangeMs = now;

  bool pressed = (digitalRead(config::ROT_BTN) == LOW);
  if (pressed != rotaryButtonPressed) {
    rotaryButtonPressed = pressed;
    if (!pressed) {
      rotaryButtonClicked = true;
    }
  }
}

void updateJoystickButton() {
  bool pressed = (digitalRead(config::JOY_BTN) == LOW);
  if (pressed && !lastJoystickState) {
    joystickClick = true;
  }
  lastJoystickState = pressed;
}

int applySoftAcceleration(int rawDelta) {
  if (rawDelta == 0) {
    return 0;
  }

  uint32_t now = millis();
  uint32_t elapsed = now - lastEncoderEventMs;
  lastEncoderEventMs = now;

  if (elapsed > kAccelerationResetMs) {
    encoderAccelerationRemainder = 0.0f;
  }

  float factor = 1.0f;
  if (elapsed <= 40) {
    factor = 2.0f;
  } else if (elapsed <= 100) {
    factor = 1.6f;
  } else if (elapsed <= 180) {
    factor = 1.3f;
  }

  float scaled = static_cast<float>(rawDelta) * factor + encoderAccelerationRemainder;
  int accelerated = static_cast<int>(scaled);
  encoderAccelerationRemainder = scaled - static_cast<float>(accelerated);

  if (accelerated == 0) {
    accelerated = (rawDelta > 0) ? 1 : -1;
    encoderAccelerationRemainder = scaled - static_cast<float>(accelerated);
  }

  if (accelerated > kMaxAcceleratedStep) {
    encoderAccelerationRemainder += static_cast<float>(accelerated - kMaxAcceleratedStep);
    accelerated = kMaxAcceleratedStep;
  } else if (accelerated < -kMaxAcceleratedStep) {
    encoderAccelerationRemainder += static_cast<float>(accelerated + kMaxAcceleratedStep);
    accelerated = -kMaxAcceleratedStep;
  }

  return accelerated;
}

}  // namespace

namespace input {

void init() {
  pinMode(config::JOY_BTN, INPUT_PULLUP);
  pinMode(config::ROT_A, INPUT_PULLUP);
  pinMode(config::ROT_B, INPUT_PULLUP);
  pinMode(config::ROT_BTN, INPUT_PULLUP);

  rotaryEncoder.begin();
  rotaryEncoder.setup(handleEncoderISR, handleEncoderButtonISR);
  ConfigureEncoderSteps(rotaryEncoder);
  ConfigureEncoderBoundaries(rotaryEncoder);
  rotaryEncoder.disableAcceleration();
  rotaryEncoder.reset(0);

  lastEncoderValue = ReadEncoderValue(rotaryEncoder);
  lastEncoderEventMs = millis();
  encoderAccelerationRemainder = 0.0f;

  rotaryButtonPressed = (digitalRead(config::ROT_BTN) == LOW);
  lastRotaryButtonChangeMs = millis();

  analogReadResolution(12);
}

JoystickCalibration calibrateJoystick() {
  long sumX = 0;
  long sumY = 0;
  constexpr int samples = 100;
  for (int i = 0; i < samples; ++i) {
    sumX += analogRead(config::JOY_X);
    sumY += analogRead(config::JOY_Y);
    delay(5);
  }
  currentCalibration.centerX = sumX / samples;
  currentCalibration.centerY = sumY / samples;
  return currentCalibration;
}

void update() {
  ServiceEncoder(rotaryEncoder);
  updateRotaryButton();
  updateJoystickButton();
}

float getJoystickNormalizedX() {
  int value = analogRead(config::JOY_X);
  float normalized = static_cast<float>(value - currentCalibration.centerX) / 2048.0f;
  if (fabs(normalized) < config::JOYSTICK_DEADZONE) {
    normalized = 0.0f;
  }
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < -1.0f) normalized = -1.0f;
  return normalized * static_cast<float>(config::JOYSTICK_X_DIRECTION);
}

float getJoystickNormalizedY() {
  int value = analogRead(config::JOY_Y);
  float normalized = static_cast<float>(value - currentCalibration.centerY) / 2048.0f;
  if (fabs(normalized) < config::JOYSTICK_DEADZONE) {
    normalized = 0.0f;
  }
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < -1.0f) normalized = -1.0f;
  return normalized * static_cast<float>(config::JOYSTICK_Y_DIRECTION);
}

bool consumeJoystickPress() {
  bool clicked = joystickClick;
  joystickClick = false;
  return clicked;
}

bool isJoystickButtonPressed() { return lastJoystickState; }

int consumeEncoderDelta() {
  long current = ReadEncoderValue(rotaryEncoder);
  int rawDelta = static_cast<int>(current - lastEncoderValue);
  if (rawDelta == 0) {
    return 0;
  }
  lastEncoderValue = current;

  if (rawDelta > kMaxAcceleratedStep || rawDelta < -kMaxAcceleratedStep) {
    encoderAccelerationRemainder = 0.0f;
    return rawDelta;
  }

  int accelerated = applySoftAcceleration(rawDelta);
  return accelerated;
}

bool consumeEncoderClick() {
  bool clicked = rotaryButtonClicked;
  rotaryButtonClicked = false;
  return clicked;
}

int getJoystickCenterX() { return currentCalibration.centerX; }

int getJoystickCenterY() { return currentCalibration.centerY; }

void setJoystickCalibration(const JoystickCalibration& calibration) {
  currentCalibration = calibration;
}

}  // namespace input

#endif  // DEVICE_ROLE_HID

