#ifndef AUDIO_CONTROLLER_H
#define AUDIO_CONTROLLER_H

#include <Arduino.h>

#include "config.h"
#include "context.h"

// Drives the DFPlayer PRO (DFR0768) on Serial2 to play spoken-prompt clips tied to
// machine events. Like LedController, this is a pure OBSERVER of SystemContext and is
// serviced from main.ino's loop() (never the 1 kHz tick): it watches the state/door/
// lock/power fields for edges and fires the matching clip once per transition.
//
// Playback is fire-and-forget: it writes the raw AT command (AT+PLAYNUM=n) and does
// NOT wait for the module's ACK. The DFRobot_DF1201S library's playFileNum() blocks
// up to 1 s on the ACK, which could stall the loop and trip the ESC comms watchdog
// mid-spin -- so the library is used only for one-time setup in begin().
class AudioController {
 public:
  void begin();
  // Observe ctx for event edges + service manual play/volume requests. Reads and
  // clears ctx.audioManualReq / ctx.audioVolPending; publishes ctx.audioReady.
  void service(SystemContext &ctx, uint32_t nowMs);

 private:
#if defined(ARDUINO_ARCH_ESP32) && defined(HAS_AUDIO) && HAS_AUDIO
  bool tryInit();          // (re)handshake + configure the module; sets ready_ on success
  void enqueue(uint8_t fileNum);  // queue a clip (spaced by AUDIO_CUE_GAP_MS in service)
  void playFile(uint8_t fileNum); // low-level: fire the raw AT+PLAYNUM now
  void setVolume(uint8_t vol);

  bool ready_ = false;   // module initialized OK -> playback enabled
  bool primed_ = false;  // captured the baseline ctx so boot state doesn't fire a cue
  uint32_t lastInitMs_ = 0;  // last idle re-probe timestamp (throttles retries)

  // Small FIFO so bursts of cues are spaced out instead of interrupting each other.
  static const uint8_t kQueueLen = 8;
  uint8_t queue_[kQueueLen] = {0};
  uint8_t qHead_ = 0;
  uint8_t qCount_ = 0;
  uint32_t lastPlayMs_ = 0;

  // Last-observed context fields, for edge detection.
  SystemState lastState_ = STATE_BOOT;
  DoorMotorCommand lastDoorCmd_ = DOOR_MOTOR_STOP;
  bool lastLock_ = true;      // gantry starts LOCKED (matches state_machine begin())
  bool lastPowered_ = false;

  // Lock-cue debounce: the lock flag can flip fast (rpm safety net vs jitter), so only
  // announce it once stable. lastLock_ is the last ANNOUNCED state; lockCandidate_ tracks
  // the latest raw value and lockChangeMs_ when it last changed.
  bool lockCandidate_ = true;
  uint32_t lockChangeMs_ = 0;
  bool inFault_ = false;      // true while HARD_STOP/FAULT_LATCHED (narration suppressed)
#endif
};

#endif  // AUDIO_CONTROLLER_H
