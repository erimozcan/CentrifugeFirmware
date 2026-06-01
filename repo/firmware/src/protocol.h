#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

#include "context.h"

namespace Protocol {

bool parseInt32(const char *text, int32_t &value);
bool parseKeyValueInt(const char *token, const char *key, int32_t &value);

bool sendOk(int32_t seq, const char *payload);
bool sendErr(int32_t seq, const char *code);
bool sendStatus(int32_t seq, const SystemContext &ctx);

}  // namespace Protocol

#endif
