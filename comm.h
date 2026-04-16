#pragma once

#include <Arduino.h>

#include <vector>

#include "config.h"

namespace comm {

struct Request {
  uint16_t id;
  String command;
  std::vector<String> params;
};

void initLink();
void updateLink();

void announceReady();
bool readRequest(Request& request,
                 uint32_t timeoutMs = config::COMM_RESPONSE_TIMEOUT_MS);
bool hasRequest();
Request nextRequest();
void sendResponse(uint16_t id, std::initializer_list<String> payload = {});
void sendOk(uint16_t id, std::initializer_list<String> payload = {});
void sendError(uint16_t id, const String& message);

}  // namespace comm

