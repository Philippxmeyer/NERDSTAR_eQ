#include "comm.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <algorithm>
#include <deque>
#include <utility>

#if defined(DEVICE_ROLE_HID)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#endif

#include "Comms.h"
#include "debug.h"

namespace {

HardwareSerial uartLink(static_cast<int>(config::COMM_UART_NUM));
Comms commsLink;
uint16_t nextRequestId = 1;
#if defined(DEVICE_ROLE_HID)
SemaphoreHandle_t rpcMutex = nullptr;
constexpr uint8_t kMaxCallRetries = 3;
#endif

constexpr uint8_t kAsciiChannel = 1;
constexpr size_t kMaxQueuedMessages = 16;

enum class MessageType : uint8_t { kReady, kRequest, kResponse, kUnknown };

struct WireMessage {
  MessageType type = MessageType::kUnknown;
  uint16_t id = 0;
  String command;
  String status;
  std::vector<String> params;
};

std::deque<WireMessage> messageQueue;

void pumpLink() { commsLink.update(); }

void dropPendingInput() {
  messageQueue.clear();
  commsLink.clearError();
}

MessageType parseType(const String& token) {
  if (token == "READY") {
    return MessageType::kReady;
  }
  if (token == "REQ") {
    return MessageType::kRequest;
  }
  if (token == "RESP") {
    return MessageType::kResponse;
  }
  return MessageType::kUnknown;
}

WireMessage parseLine(const String& line) {
  WireMessage message;
  std::vector<String> fields;
  int start = 0;
  while (start <= line.length()) {
    int idx = line.indexOf('|', start);
    if (idx < 0) {
      fields.push_back(line.substring(start));
      break;
    }
    fields.push_back(line.substring(start, idx));
    start = idx + 1;
  }

  if (fields.empty()) {
    return message;
  }

  message.type = parseType(fields[0]);
  switch (message.type) {
    case MessageType::kReady:
      break;
    case MessageType::kRequest:
      if (fields.size() >= 3) {
        message.id = static_cast<uint16_t>(fields[1].toInt());
        message.command = fields[2];
        message.params.assign(fields.begin() + 3, fields.end());
      }
      break;
    case MessageType::kResponse:
      if (fields.size() >= 3) {
        message.id = static_cast<uint16_t>(fields[1].toInt());
        message.status = fields[2];
        message.params.assign(fields.begin() + 3, fields.end());
      }
      break;
    case MessageType::kUnknown:
    default:
      break;
  }

  if (message.type == MessageType::kRequest &&
      (message.command.isEmpty() || fields.size() < 3)) {
    message.type = MessageType::kUnknown;
  }
  if (message.type == MessageType::kResponse &&
      (message.status.isEmpty() || fields.size() < 3)) {
    message.type = MessageType::kUnknown;
  }

  return message;
}

String buildLine(const WireMessage& message) {
  switch (message.type) {
    case MessageType::kReady:
      return String("READY");
    case MessageType::kRequest: {
      String line = "REQ|" + String(message.id) + "|" + message.command;
      for (const auto& param : message.params) {
        line += "|";
        line += param;
      }
      return line;
    }
    case MessageType::kResponse: {
      String line = "RESP|" + String(message.id) + "|" + message.status;
      for (const auto& param : message.params) {
        line += "|";
        line += param;
      }
      return line;
    }
    case MessageType::kUnknown:
    default:
      return String();
  }
}

bool sendMessage(const WireMessage& message) {
  String line = buildLine(message);
  if (line.isEmpty()) {
    return false;
  }
  const size_t length = static_cast<size_t>(line.length());
  if (length > Comms::kMaxPayloadSize) {
    if (Serial) {
      Serial.println("[COMM] TX line too long for packet buffer");
    }
    return false;
  }
  if (!commsLink.send(kAsciiChannel,
                      reinterpret_cast<const uint8_t*>(line.c_str()), length)) {
    if (Serial) {
      Serial.println("[COMM] Failed to queue packet for transmission");
    }
    return false;
  }
  return true;
}

bool readMessage(WireMessage& message, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (true) {
    pumpLink();
    if (!messageQueue.empty()) {
      message = messageQueue.front();
      messageQueue.pop_front();
      return true;
    }
    if (timeoutMs != 0) {
      uint32_t now = millis();
      if ((now - start) >= timeoutMs) {
        return false;
      }
    }
    delay(1);
  }
}

void enqueueMessage(WireMessage&& message) {
  if (message.type == MessageType::kUnknown) {
    return;
  }
  if (message.type == MessageType::kReady) {
    for (const auto& pending : messageQueue) {
      if (pending.type == MessageType::kReady) {
        return;  // Already queued a READY notification.
      }
    }
  }

  if (messageQueue.size() >= kMaxQueuedMessages) {
    auto readyIt = std::find_if(messageQueue.begin(), messageQueue.end(),
                                [](const WireMessage& msg) {
                                  return msg.type == MessageType::kReady;
                                });
    if (readyIt != messageQueue.end()) {
      messageQueue.erase(readyIt);
    } else {
      messageQueue.pop_front();
    }
  }

  messageQueue.push_back(std::move(message));
}

void handlePacket(const Comms::Packet& packet, void*) {
  if (packet.channel != kAsciiChannel) {
    return;
  }
  String line;
  line.reserve(packet.size);
  for (uint8_t i = 0; i < packet.size; ++i) {
    line += static_cast<char>(packet.data[i]);
  }
  enqueueMessage(parseLine(line));
}

void handleHeartbeat(void*) {
  // Nothing to do; link state is tracked inside Comms.
}

void handleError(Comms::Error error, int8_t rawStatus, void*) {
  if (!Serial) {
    return;
  }
  switch (error) {
    case Comms::Error::kPayloadTooLarge:
      Serial.println("[COMM] Payload too large for TX buffer");
      break;
    case Comms::Error::kInvalidPayload:
      Serial.println("[COMM] Invalid payload pointer");
      break;
    case Comms::Error::kHeartbeatLost:
      Serial.println("[COMM] Heartbeat lost");
      break;
    case Comms::Error::kSerialTransfer:
      Serial.printf("[COMM] SerialTransfer error: %d\n", rawStatus);
      break;
    case Comms::Error::kNone:
    default:
      break;
  }
}

}  // namespace

namespace comm {

void initLink() {
  messageQueue.clear();
  nextRequestId = 1;
  commsLink.begin(uartLink, config::COMM_RX_PIN, config::COMM_TX_PIN,
                  config::COMM_BAUD);
  commsLink.setHeartbeatInterval(config::COMM_HEARTBEAT_INTERVAL_MS);
  commsLink.setHeartbeatTimeout(config::COMM_HEARTBEAT_TIMEOUT_MS);

  Comms::Callbacks callbacks{};
  callbacks.onPacket = handlePacket;
  callbacks.onHeartbeat = handleHeartbeat;
  callbacks.onError = handleError;
  commsLink.setCallbacks(callbacks);
  commsLink.clearError();
#if defined(DEVICE_ROLE_HID)
  if (rpcMutex == nullptr) {
    rpcMutex = xSemaphoreCreateMutex();
  }
#endif
}

void updateLink() { pumpLink(); }

#if defined(DEVICE_ROLE_HID)

bool waitForReady(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (true) {
    uint32_t remaining = 0;
    if (timeoutMs != 0) {
      uint32_t elapsed = millis() - start;
      if (elapsed >= timeoutMs) {
        return false;
      }
      remaining = timeoutMs - elapsed;
    }
    WireMessage message;
    if (!readMessage(message, remaining)) {
      return false;
    }
    if (message.type == MessageType::kReady) {
      return true;
    }
  }
}

bool call(const char* command, std::initializer_list<String> params,
          std::vector<String>* payload, String* error, uint32_t timeoutMs) {
  if (rpcMutex == nullptr) {
    rpcMutex = xSemaphoreCreateMutex();
  }
  class MutexLock {
   public:
    explicit MutexLock(SemaphoreHandle_t handle) : handle_(handle), locked_(false) {
      if (!handle_) {
        return;
      }
      BaseType_t schedulerState = xTaskGetSchedulerState();
      TickType_t wait =
          (schedulerState == taskSCHEDULER_NOT_STARTED || xPortInIsrContext())
              ? 0
              : portMAX_DELAY;
      locked_ = xSemaphoreTake(handle_, wait) == pdTRUE;
    }
    ~MutexLock() {
      if (locked_ && handle_) {
        xSemaphoreGive(handle_);
      }
    }
    bool locked() const { return locked_; }

   private:
    SemaphoreHandle_t handle_;
    bool locked_;
  } lock(rpcMutex);
  if (!lock.locked()) {
    if (error) {
      *error = "Mutex";
    }
    return false;
  }
  String lastError = "Timeout";
  for (uint8_t attempt = 0; attempt < kMaxCallRetries; ++attempt) {
    debug::recordCommAttempt(command);
    if (payload) {
      payload->clear();
    }
    uint16_t id = nextRequestId++;
    WireMessage request{MessageType::kRequest, id, String(command), String(),
                       std::vector<String>(params)};
    if (attempt > 0 && Serial) {
      Serial.printf("[COMM] Retrying %s (attempt %u, last error: %s)\n", command,
                    attempt + 1, lastError.c_str());
    }
    if (!sendMessage(request)) {
      lastError = "Send";
      break;
    }

    uint32_t start = millis();
    while (true) {
      uint32_t remaining = 0;
      if (timeoutMs != 0) {
        uint32_t elapsed = millis() - start;
        if (elapsed >= timeoutMs) {
          lastError = "Timeout";
          break;
        }
        remaining = timeoutMs - elapsed;
      }
      WireMessage response;
      if (!readMessage(response, remaining)) {
        lastError = "Timeout";
        break;
      }
      if (response.type == MessageType::kReady) {
        continue;
      }
      if (response.type != MessageType::kResponse || response.id != id) {
        lastError = "Protocol";
        continue;
      }
      if (response.status == "OK") {
        if (payload) {
          payload->assign(response.params.begin(), response.params.end());
        }
        debug::recordCommSuccess(command);
        return true;
      }
      lastError = response.params.empty() ? String("Error") : response.params[0];
      break;
    }

    dropPendingInput();
    if (lastError != "Timeout" && lastError != "Protocol") {
      break;
    }
    waitForReady(200);
  }

  if (error) {
    *error = lastError;
  }
  debug::recordCommFailure(command, lastError.c_str());
  if (Serial) {
    Serial.printf("[COMM] Command %s failed after %u attempts: %s\n", command,
                  static_cast<unsigned>(kMaxCallRetries), lastError.c_str());
  }
  return false;
}

bool isLinkActive() {
  pumpLink();
  return commsLink.isActive();
}

#elif defined(DEVICE_ROLE_MAIN)

void announceReady() {
  WireMessage message;
  message.type = MessageType::kReady;
  sendMessage(message);
}

bool readRequest(Request& request, uint32_t timeoutMs) {
  while (true) {
    WireMessage message;
    if (!readMessage(message, timeoutMs)) {
      return false;
    }
    if (message.type == MessageType::kReady) {
      continue;
    }
    if (message.type != MessageType::kRequest || message.command.isEmpty()) {
      continue;
    }
    request.id = message.id;
    request.command = message.command;
    request.params = std::move(message.params);
    return true;
  }
}

void sendOk(uint16_t id, std::initializer_list<String> payload) {
  WireMessage message;
  message.type = MessageType::kResponse;
  message.id = id;
  message.status = "OK";
  message.params.assign(payload.begin(), payload.end());
  sendMessage(message);
}

void sendError(uint16_t id, const String& message) {
  WireMessage response;
  response.type = MessageType::kResponse;
  response.id = id;
  response.status = "ERR";
  response.params.push_back(message);
  sendMessage(response);
}

#endif

}  // namespace comm

