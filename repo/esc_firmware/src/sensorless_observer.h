#ifndef SENSORLESS_OBSERVER_H
#define SENSORLESS_OBSERVER_H

#include <Arduino.h>

struct SensorlessObserverConfig {
  float polePairs;
  float phaseResistanceOhm;
  float phaseInductanceH;
  float kvRpmPerVolt;
  float pllKp;
  float pllKi;
  float minBemfVolts;
  float lockPhaseErrorRad;
  float lockRpmError;
  float lockDwellMs;
  float maxRpm;
};

class SensorlessObserver {
 public:
  void begin(const SensorlessObserverConfig &cfg);
  void resetFromEncoder(float mechanicalAngleRad, float mechanicalRpm);
  void resetFromElectrical(float electricalAngleRad, float mechanicalRpm);
  void update(float ia, float ib, float ic,
              float uq, float ud, float voltageAngleElectrical,
              float encoderElectricalAngle, float encoderMechanicalRpm,
              float dt);

  float angleElectrical() const { return electricalAngle_; }
  float rpmMechanical() const { return mechanicalRpm_; }
  bool locked() const { return locked_; }
  float phaseErrorToEncoder() const { return phaseErrorToEncoder_; }
  float bemfVolts() const { return bemfVolts_; }

 private:
  static float wrapPi(float angle);
  static float normalizeAngle(float angle);

  SensorlessObserverConfig cfg_{};
  bool configured_ = false;
  bool locked_ = false;
  float electricalAngle_ = 0.0f;
  float electricalOmega_ = 0.0f;
  float mechanicalRpm_ = 0.0f;
  float lastIalpha_ = 0.0f;
  float lastIbeta_ = 0.0f;
  float lockMs_ = 0.0f;
  float phaseErrorToEncoder_ = 0.0f;
  float bemfVolts_ = 0.0f;
};

#endif
