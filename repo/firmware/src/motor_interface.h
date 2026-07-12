#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <Arduino.h>

// Bridges the Due state machine to the B-G431B-ESC1 over Serial1 using the ESC's
// high-level verb protocol (SPIN <rpm> / STOP / ESTOP / PING / STATUS?). The ESC
// owns FOC/align/ramp; the Due just speaks intent. We translate the per-tick
// ramped setpoint into a paced verb stream and parse the ESC's ST telemetry so
// the Due reports the ESC's actual measured RPM.
class MotorInterface {
 public:
  void begin();
  void setCommandOutputEnabled(bool enabled);
  void sendVelocitySetpoint(int32_t rpm);
  void applyEscTuning();          // push tuned closed-loop gains to the ESC (see .cpp)
  void drainRx();

  // Gantry indexing via the ESC's native closed-loop angle move (ROTATE_VIA_INDEX).
  void commandIndex(uint8_t escTube);   // send INDEX on entry + PING heartbeat; call each tick
  void finishIndexIfNeeded();           // if indexing, send STOP once to disarm the ESC hold
  bool indexArrived() const;            // ESC reported it reached the tube (state=indexed)

  // Gantry indexing via the NANO closing the loop over the ESC's SPIN + encoder ANGLE.
  // Crawls forward to an ABSOLUTE detent (works from any position, e.g. a random post-spin
  // stop). Detents are learned relative to a homed reference.
  void requestHome();                     // capture the current shaft angle as detent 0 (tube 1)
  void maintainHome();                    // call when idle: completes a pending home capture
  bool homeSet() const;
  void startRotateToTube(uint8_t tube);   // crawl to tube 1..TUBE_COUNT (absolute detent)
  void startRotateToNearestDetent();      // crawl to the nearest detent (post-spin re-lock)
  void updateVelocityRotate();            // call each tick during the move (drives SPIN + polls)
  bool velocityRotateArrived() const;
  void endVelocityRotate();               // stop + reset (idempotent)
  uint8_t arrivedTube() const;            // tube nearest the current angle (1..N; 0 if not homed)

  int32_t lastMeasuredRpm() const;   // actual RPM parsed from ESC ST telemetry

 private:
  enum EscIntent { INTENT_IDLE = 0, INTENT_SPIN, INTENT_ESTOP };

  bool commandOutputEnabled_ = true;
  int32_t lastCommandedRpm_ = 0;     // last setpoint we were asked to command

  // TX pacing / intent tracking
  EscIntent intent_ = INTENT_IDLE;
  int32_t lastSentRpm_ = -1;
  uint32_t lastTxMs_ = 0U;

  // RX (ESC -> Due) line assembly + parsed telemetry. Sized for the ESC's long bench STAT
  // line (rev=/ang=/cnt= + phase currents, ~130 chars), which the rotate loop reads.
  char rxBuf_[160];
  uint8_t rxLen_ = 0U;
  int32_t measuredRpm_ = 0;
  uint32_t lastRxMs_ = 0U;

  // Indexing state (ROTATE_VIA_INDEX)
  bool indexActive_ = false;         // an INDEX has been issued and not yet finished
  bool indexArrived_ = false;        // ESC reported state=indexed
  int32_t lastIndexTube_ = -1;       // last tube we sent INDEX for (re-send on change)

  // Velocity-feedback rotate state (Nano closes the loop over SPIN + STAT ang=/rev=)
  enum RotPhase { ROT_INACTIVE = 0, ROT_ACQUIRE, ROT_CRAWL, ROT_SETTLE, ROT_DONE };
  RotPhase rotPhase_ = ROT_INACTIVE;
  bool rotNearest_ = false;          // true = go to nearest detent, false = specific tube
  float rotTargetFrac_ = 0.0f;       // absolute target (fraction of a rev), tube mode
  float measuredFrac_ = 0.0f;        // absolute shaft angle [0,1) from STAT ang=
  float measuredRev_ = 0.0f;         // cumulative revs from STAT rev= (motion/settle only)
  bool rotSeen_ = false;             // a STAT reading arrived since the move/home started
  float rotLastRev_ = 0.0f;
  uint32_t rotStableMs_ = 0U;
  uint32_t rotPollMs_ = 0U;
  // Detent reference (learned by homing): absolute frac of detent 0 (tube 1).
  float detentRef_ = 0.0f;
  bool homeSet_ = false;
  bool homePending_ = false;

  void txSpin(int32_t rpm);
  void txIndex(uint8_t escTube);
  void txLiteral(const char *verb);
  void applyRotateTuning();   // boost velocity P for low-speed torque during a crawl
  void parseEscLine(const char *line);

  static size_t writeInt32(char *out, size_t maxLen, int32_t value);
};

#endif
