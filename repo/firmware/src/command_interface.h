#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include <Arduino.h>

#include "context.h"

class CommandInterface {
 public:
  void begin();
  void poll(PendingCommand &pending, const SystemContext &ctx);

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
