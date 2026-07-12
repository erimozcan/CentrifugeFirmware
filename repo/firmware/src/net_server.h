#ifndef NET_SERVER_H
#define NET_SERVER_H

#include <Arduino.h>

#include "config.h"

// WiFi operator-console server. Runs entirely on core 0 as its own task: a WiFi soft-AP,
// an HTTP server (serves the embedded UI + render clips), and a WebSocket server for the
// command protocol. It NEVER touches the state machine or shared context directly -- it
// exchanges data with the control loop (core 1) only through thread-safe FreeRTOS queues,
// so the motor-control tick and ESC link are completely isolated from networking.
//
// Flow: browser --WS--> netTask (core0) --cmdQueue--> loop (core1) processes the command
//       via the SAME CommandInterface as USB --replyQueue--> netTask --WS--> browser.
namespace NetServer {

struct Cmd {
  uint8_t client;
  char line[96];
};

void begin();                                   // start the AP + servers (call from setup)
bool popCommand(Cmd &out);                      // core1: dequeue one inbound command
void pushReply(uint8_t client, const char *line);  // core1: queue a reply line for a client
uint8_t clients();                              // number of connected WebSocket clients

}  // namespace NetServer

// A Print sink that captures a command's reply into a fixed buffer, so it can be sent back
// over the WebSocket instead of Serial (see Protocol::setOutput).
class LineCapture : public Print {
 public:
  size_t write(uint8_t c) override {
    if (len_ < sizeof(buf_) - 1U) buf_[len_++] = static_cast<char>(c);
    return 1;
  }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n; ++i) write(b[i]);
    return n;
  }
  const char *c_str() { buf_[len_] = '\0'; return buf_; }
  size_t length() const { return len_; }
  void reset() { len_ = 0; }

 private:
  char buf_[256];
  size_t len_ = 0;
};

#endif  // NET_SERVER_H
