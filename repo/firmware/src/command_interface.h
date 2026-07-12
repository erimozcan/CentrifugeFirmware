#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include <Arduino.h>

#include "context.h"

class CommandInterface {
 public:
  void begin();
  void poll(PendingCommand &pending, const SystemContext &ctx);

  // Process one already-assembled command line from a non-Serial source (e.g. the WiFi
  // WebSocket bridge). `line` must be a writable, NUL-terminated buffer (it is tokenized
  // in place). Replies go to the current Protocol output sink. Must be called on the main
  // loop (core 1), same as poll(), so ctx/pending access stays single-threaded.
  void handleExternalLine(char *line, PendingCommand &pending, const SystemContext &ctx) {
    handleLine(line, pending, ctx);
  }

 private:
  static const uint8_t kMaxLineLength = 127U;
  char lineBuffer_[kMaxLineLength + 1U];
  uint8_t lineLength_;

  void handleLine(char *line, PendingCommand &pending, const SystemContext &ctx);
  bool parseRunProfile(char *savePtr, RunProfile &profile);

  void queueRun(PendingCommand &pending, const RunProfile &profile);
  void queueFlag(PendingCommand &pending, bool PendingCommand::*flag);

  bool isAbortAllowed(SystemState state) const;
};

#endif
