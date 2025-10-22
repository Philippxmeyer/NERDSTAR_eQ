#include "comm.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <algorithm>
#include <deque>
#include <utility>

#if defined(DEVICE_ROLE_HID)
#include "freertos/FreeRTOS.h"
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
constexpr size_t kMaxQueuedLines = 16;

std::deque<String> lineQueue;

void pumpLink() { commsLink.update(); }

void dropPendingInput() {
  lineQueue.clear();
  commsLink.clearError();
}

bool readLine(String& line, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (true) {
    pumpLink();
    if (!lineQueue.empty()) {
      line = lineQueue.front();
      lineQueue.pop_front();
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

bool sendLine(const String& line) {
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

void handlePacket(const Comms::Packet& packet, void*) {
  if (packet.channel != kAsciiChannel) {
    return;
  }
  String line;
  line.reserve(packet.size);
  for (uint8_t i = 0; i < packet.size; ++i) {
    line += static_cast<char>(packet.data[i]);
  }
  if (line.equals("READY")) {
    for (const auto& existing : lineQueue) {
      if (existing.equals("READY")) {
        return;  // Already queued a READY notification.
      }
    }
  }
  if (lineQueue.size() >= kMaxQueuedLines) {
    auto readyIt = std::find_if(lineQueue.begin(), lineQueue.end(), [](const String& value) {
      return value.equals("READY");
    });
    if (readyIt != lineQueue.end()) {
      lineQueue.erase(readyIt);
    } else {
      lineQueue.pop_front();
    }
  }
  lineQueue.push_back(std::move(line));
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

void splitFields(const String& line, std::vector<String>& fields) {
  fields.clear();
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
}

}  // namespace

namespace comm {

void initLink() {
  lineQueue.clear();
  nextRequestId = 1;
  commsLink.begin(uartLink, config::COMM_RX_PIN, config::COMM_TX_PIN,
                  config::COMM_BAUD);
  commsLink.setHeartbeatInterval(50);
  commsLink.setHeartbeatTimeout(500);

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
    String line;
    if (!readLine(line, remaining)) {
      return false;
    }
    if (line == "READY") {
      return true;
    }
  }
}

bool call(const char* command, std::initializer_list<String> params,
          std::vector<String>* payload, String* error, uint32_t timeoutMs) {
  class MutexLock {
   public:
    explicit MutexLock(SemaphoreHandle_t handle) : handle_(handle), locked_(false) {
      if (handle_) {
        locked_ = xSemaphoreTake(handle_, portMAX_DELAY) == pdTRUE;
      }
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
    String line = "REQ|" + String(id) + "|" + String(command);
    for (const auto& param : params) {
      line += "|";
      line += param;
    }
    if (attempt > 0 && Serial) {
      Serial.printf("[COMM] Retrying %s (attempt %u, last error: %s)\n", command,
                    attempt + 1, lastError.c_str());
    }
    if (!sendLine(line)) {
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
      String response;
      if (!readLine(response, remaining)) {
        lastError = "Timeout";
        break;
      }
      if (response == "READY") {
        continue;
      }
      std::vector<String> fields;
      splitFields(response, fields);
      if (fields.empty()) {
        lastError = "Protocol";
        continue;
      }
      if (fields[0] != "RESP") {
        lastError = "Protocol";
        continue;
      }
      if (fields.size() < 3) {
        lastError = "Protocol";
        continue;
      }
      uint16_t respId = static_cast<uint16_t>(fields[1].toInt());
      if (respId != id) {
        lastError = "Protocol";
        continue;
      }
      const String& status = fields[2];
      if (status == "OK") {
        if (payload) {
          payload->assign(fields.begin() + 3, fields.end());
        }
        debug::recordCommSuccess(command);
        return true;
      }
      if (fields.size() > 3) {
        lastError = fields[3];
      } else {
        lastError = "Error";
      }
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

void announceReady() { sendLine("READY"); }

bool readRequest(Request& request, uint32_t timeoutMs) {
  while (true) {
    String line;
    if (!readLine(line, timeoutMs)) {
      return false;
    }
    if (line == "READY") {
      continue;
    }
    std::vector<String> fields;
    splitFields(line, fields);
    if (fields.empty()) {
      continue;
    }
    if (fields[0] != "REQ") {
      continue;
    }
    if (fields.size() < 3) {
      continue;
    }
    request.id = static_cast<uint16_t>(fields[1].toInt());
    request.command = fields[2];
    request.params.assign(fields.begin() + 3, fields.end());
    return true;
  }
}

void sendOk(uint16_t id, std::initializer_list<String> payload) {
  String line = "RESP|" + String(id) + "|OK";
  for (const auto& value : payload) {
    line += "|";
    line += value;
  }
  sendLine(line);
}

void sendError(uint16_t id, const String& message) {
  String line = "RESP|" + String(id) + "|ERR|" + message;
  sendLine(line);
}

#endif

}  // namespace comm

