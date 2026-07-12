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

// Reply sink. Command replies (OK/ERR/STATUS) are written here; defaults to Serial
// (the USB path). The WiFi bridge temporarily redirects it to a String-capturing Print
// so a WebSocket command's reply can be captured and sent back over the socket. All
// command processing happens on the main loop (core 1), so switching the sink per
// command is race-free w.r.t. the Serial path.
void setOutput(Print *out);   // pass nullptr to restore Serial
Print *output();

}  // namespace Protocol

#endif
