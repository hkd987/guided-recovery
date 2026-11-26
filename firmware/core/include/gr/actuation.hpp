// Turns a commanded turn rate into left/right brake positions, and drives the
// two winches to those positions with stall detection.
#pragma once

#include "gr/math.hpp"
#include "gr/params.hpp"

namespace gr {

struct BrakeCmd {
  float left = 0.0f;   // 0..1 of full brake travel
  float right = 0.0f;
};

// Turn-rate loop: feedforward from the canopy's expected turn response plus
// PI on the measured turn rate. The integrator trims out a canopy that pulls
// a little to one side.
//
// Separately, a two-state Kalman filter learns the canopy's turn response
// from the measured winch positions:  turnRate = K * diff + bias.
// The brake asymmetry it needs just to fly straight is -bias/K; if that
// exceeds lopsidedTrim the canopy is declared lopsided. Learning K as well
// keeps a canopy that merely turns weakly from looking lopsided.
//
// That test needs the brakes to move both ways before it is confident. A
// canopy pulling so hard that one brake stays pinned never gives it that, so
// a second rule catches it directly: brake held near full travel, and the
// canopy still turning far less than asked, for several seconds.
class Steering {
 public:
  Steering(const SteeringParams& s, const CanopyParams& c) : s_(s), c_(c) {}
  void reset();
  // actualDiff: right minus left brake, from the winch encoders (0..1 units).
  BrakeCmd update(float turnRateCmd, float symBrake, float yawRateMeas, float actualDiff, float dt);
  bool lopsided() const { return lopsided_; }
  float trim() const { return integ_; }
  // Brake asymmetry (right minus left) needed to fly straight.
  float trimNeeded() const { return -th1_ / th0_; }
  float learnedTurnGain() const { return th0_; }
  // 1-sigma uncertainty of trimNeeded(). Large until the brakes have moved
  // both ways enough to tell "weak canopy" from "canopy pulls to one side".
  float trimNeededSd() const;
  float filteredYawRate() const { return rate_.value(); }

 private:
  SteeringParams s_;
  CanopyParams c_;
  LowPass rate_{0.25f};
  LowPass diffLag_{0.6f};   // canopy turn response lags the brakes
  LowPass diffFast_{0.1f};
  float th0_ = 0.9f, th1_ = 0.0f;  // learned K and bias
  float p00_ = 0.06f, p01_ = 0.0f, p11_ = 0.09f;  // K known to ~27 % (canopy spread), bias not
  float accD_ = 0.0f, accR_ = 0.0f, accT_ = 0.0f;  // 20 Hz averaging
  int accN_ = 0;
  float integ_ = 0.0f;
  float sinceReset_ = 0.0f;
  float lopsidedT_ = 0.0f;
  float stuckT_ = 0.0f;
  float eff_[2] = {0.0f, 0.0f};    // best turn per unit brake seen each way: right, left
  float heldT_[2] = {0.0f, 0.0f};  // time the brake has been held past 40 % each way
  float pinT_ = 0.0f, pinDir_ = 0.0f;  // continuous time pinned past stuckDiff, and which way
  bool lopsided_ = false;
};

// Mixes symmetric + differential brake into per-side positions, preserving
// the differential (turn) part when one side would exceed full travel.
BrakeCmd mixBrakes(float symBrake, float diffBrake);

// Position loop for one winch: brake fraction -> line travel (mm) -> duty.
class WinchController {
 public:
  void configure(const WinchParams& p, int channel);
  void reset();
  // Returns duty in -1..1 (positive pulls line in). With powered = false the
  // controller only tracks position: no exercise, no stall judgement.
  float update(float targetFrac, int32_t encoderCount, float batteryV, float dt, bool powered = true);
  float positionMm() const { return posMm_; }
  float targetMm() const { return targetMm_; }
  // A jam, or the motor driving away from the target (motor and encoder
  // wired with opposite polarity). Either way the motor is stopped.
  bool stalled() const { return stalled_; }
  bool wrongWay() const { return wrongWay_; }
  void clearFault() { stalled_ = false; wrongWay_ = false; stallT_ = 0.0f; wrongT_ = 0.0f; }

 private:
  WinchParams p_;
  int ch_ = 0;
  bool init_ = false;
  float posMm_ = 0.0f;
  float lastPosMm_ = 0.0f;
  LowPass vel_{0.03f};
  float targetMm_ = 0.0f;
  float lastBaseMm_ = 0.0f;
  float lastMovePosMm_ = 0.0f;  // spool position when it last actually moved
  float idleT_ = 0.0f;
  float exerciseT_ = -1.0f;
  float stallT_ = 0.0f;
  float wrongT_ = 0.0f;
  bool stalled_ = false;
  bool wrongWay_ = false;
};

}  // namespace gr
