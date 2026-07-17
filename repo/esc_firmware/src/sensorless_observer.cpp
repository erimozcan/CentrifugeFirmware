#include "sensorless_observer.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kSqrt3 = 1.73205080756887729353f;
}

void SensorlessObserver::begin(const SensorlessObserverConfig &cfg) {
  cfg_ = cfg;
  configured_ = true;
  locked_ = false;
  electricalAngle_ = 0.0f;
  electricalOmega_ = 0.0f;
  mechanicalRpm_ = 0.0f;
  lastIalpha_ = 0.0f;
  lastIbeta_ = 0.0f;
  lockMs_ = 0.0f;
  phaseErrorToEncoder_ = 0.0f;
  bemfVolts_ = 0.0f;
}

void SensorlessObserver::reset() {
  locked_ = false;
  electricalAngle_ = 0.0f;
  electricalOmega_ = 0.0f;
  mechanicalRpm_ = 0.0f;
  lastIalpha_ = 0.0f;
  lastIbeta_ = 0.0f;
  lockMs_ = 0.0f;
  phaseErrorToEncoder_ = 0.0f;
  bemfVolts_ = 0.0f;
}

void SensorlessObserver::resetFromEncoder(float mechanicalAngleRad, float mechanicalRpm) {
  resetFromElectrical(mechanicalAngleRad * cfg_.polePairs, mechanicalRpm);
}

void SensorlessObserver::resetFromElectrical(float electricalAngleRad, float mechanicalRpm) {
  electricalAngle_ = normalizeAngle(electricalAngleRad);
  mechanicalRpm_ = mechanicalRpm;
  electricalOmega_ = mechanicalRpm * cfg_.polePairs * kTwoPi / 60.0f;
  locked_ = true;
  lockMs_ = cfg_.lockDwellMs;
  phaseErrorToEncoder_ = 0.0f;
}

void SensorlessObserver::update(float ia, float ib, float ic,
                                float uq, float ud, float voltageAngleElectrical,
                                float encoderElectricalAngle, float encoderMechanicalRpm,
                                float dt) {
  (void)ic;
  if (!configured_ || dt <= 0.0f || dt > 0.02f) {
    locked_ = false;
    lockMs_ = 0.0f;
    return;
  }

  const float sinA = sinf(voltageAngleElectrical);
  const float cosA = cosf(voltageAngleElectrical);
  const float uAlpha = ud * cosA - uq * sinA;
  const float uBeta = ud * sinA + uq * cosA;

  const float iAlpha = ia;
  const float iBeta = (ia + 2.0f * ib) / kSqrt3;
  const float dIalpha = (iAlpha - lastIalpha_) / dt;
  const float dIbeta = (iBeta - lastIbeta_) / dt;
  lastIalpha_ = iAlpha;
  lastIbeta_ = iBeta;

  const float eAlpha = uAlpha - cfg_.phaseResistanceOhm * iAlpha - cfg_.phaseInductanceH * dIalpha;
  const float eBeta = uBeta - cfg_.phaseResistanceOhm * iBeta - cfg_.phaseInductanceH * dIbeta;
  bemfVolts_ = sqrtf(eAlpha * eAlpha + eBeta * eBeta);

  const float speedSign = (electricalOmega_ >= 0.0f) ? 1.0f : -1.0f;
  // The PLL's own angle stays RAW and self-consistent -- it is also the drive angle in
  // sensorless mode, and biasing it there makes the drive chase a shifted copy of
  // itself (bench: torque angle lagged ~45 deg at 4300 RPM, ~9 A, fault-rampdown).
  const float measuredElectricalAngle =
      normalizeAngle(atan2f(eBeta, eAlpha) - speedSign * (kPi * 0.5f));
  const float pllError = wrapPi(measuredElectricalAngle - electricalAngle_);

  electricalOmega_ += cfg_.pllKi * pllError * dt;
  const float maxElectricalOmega = cfg_.maxRpm * cfg_.polePairs * kTwoPi / 60.0f;
  if (electricalOmega_ > maxElectricalOmega) electricalOmega_ = maxElectricalOmega;
  if (electricalOmega_ < -maxElectricalOmega) electricalOmega_ = -maxElectricalOmega;

  electricalAngle_ = normalizeAngle(electricalAngle_ + (electricalOmega_ + cfg_.pllKp * pllError) * dt);
  mechanicalRpm_ = electricalOmega_ * 60.0f / (cfg_.polePairs * kTwoPi);

  // Frame-skew compensation lives ONLY in the supervision comparison: the observer and
  // the encoder reference are sampled through different pipelines, skewed by a fixed
  // omega * |latencySec| (bench-fitted; see cfg). Compensating here calibrates the
  // lock/unlock gate without touching the drive angle.
  phaseErrorToEncoder_ =
      wrapPi(electricalAngle_ + electricalOmega_ * cfg_.latencySec - encoderElectricalAngle);
  const float rpmError = fabsf(mechanicalRpm_ - encoderMechanicalRpm);
  const bool instantLock =
      bemfVolts_ >= cfg_.minBemfVolts &&
      fabsf(phaseErrorToEncoder_) <= cfg_.lockPhaseErrorRad &&
      rpmError <= cfg_.lockRpmError;

  if (instantLock) {
    lockMs_ += dt * 1000.0f;
    locked_ = lockMs_ >= cfg_.lockDwellMs;
  } else {
    lockMs_ = 0.0f;
    locked_ = false;
  }
}

float SensorlessObserver::wrapPi(float angle) {
  while (angle > kPi) angle -= kTwoPi;
  while (angle < -kPi) angle += kTwoPi;
  return angle;
}

float SensorlessObserver::normalizeAngle(float angle) {
  while (angle >= kTwoPi) angle -= kTwoPi;
  while (angle < 0.0f) angle += kTwoPi;
  return angle;
}
