// Recognises the host rocket's flight events with no wiring to its flight
// computer: launch, burnout, apogee, and (the important one) "canopy open".
//
// Canopy confirmation follows the spec:
//   1. ignore boost and coast,
//   2. ignore drogue descent (fast fall that never slows to parafoil speed),
//   3. look for an opening jolt, then a steady descent in the parafoil band,
//   4. only then may steering arm.
//
// "Steady" is judged on the mean descent rate over the last few seconds (from
// the altitude history), not the instantaneous rate: real logs show the rate
// swinging between about 1 and 10 m/s for 10-15 s after a canopy opens
// (pendulum swing, riser bounce), around a perfectly steady mean.
#pragma once

#include "gr/math.hpp"
#include "gr/params.hpp"

namespace gr {

enum class FlightStage : uint8_t { Disarmed = 0, Pad, Ascent, Descent, Canopy, Landed };

struct DetectorInput {
  float t = 0.0f;           // s
  bool armed = false;
  float accelG = 1.0f;      // |specific force| in g
  bool baroValid = false;
  float agl = 0.0f;         // m above the pad reference
  float climbRate = 0.0f;   // m/s, up positive
  bool mainFired = false;   // optional wired confirmation
  // Independent descent rate from GPS (Doppler), when the pod has a fix.
  bool gpsDescentValid = false;
  float gpsDescent = 0.0f;  // m/s, down positive, smoothed over ~3 s
};

class PhaseDetector {
 public:
  explicit PhaseDetector(const DetectorParams& p) : p_(p) {}
  void reset();
  // After a processor restart in flight: continue from `stage` (Ascent,
  // Descent or Canopy) with no launch candidate and burnout assumed.
  void resumeAt(FlightStage stage);
  void update(const DetectorInput& in, float dt);
  // The barometer source changed (the voter dropped one): start the mean-
  // descent history again, so no mixed data can reach a canopy decision.
  void restartDescentHistory() {
    histHead_ = histCount_ = 0;
    histAcc_ = 0.0f;
    streakActive_ = false;
  }

  FlightStage stage() const { return stage_; }
  // True from the first sign of launch (before confirmation). The pad
  // references (home, ground pressure) freeze while this is true.
  bool launchCandidate() const { return candidate_ || stage_ >= FlightStage::Ascent; }
  bool candidateFlag() const { return candidate_; }  // raw flag (the TLA+ model's `cand`)
  bool burnout() const { return burnout_; }
  float launchTime() const { return launchT_; }
  float apogeeTime() const { return apogeeT_; }
  float canopyTime() const { return canopyT_; }
  float maxAgl() const { return maxAgl_; }
  float lastJoltTime() const { return lastJoltT_; }
  float lastShockTime() const { return lastShockT_; }  // accelerometer jolts only (not the sense line)
  float steadyFor() const { return streakActive_ ? (lastT_ - streakStart_) : 0.0f; }
  float smoothedDescent() const { return descent_.value(); }
  // Mean descent rate over the last steadyWindow seconds (m/s, down positive);
  // false until the window has filled since apogee.
  bool meanDescent(float& rate) const;

 private:
  void resetStreak(float t, float d);
  void pushAltitude(float agl, float dt);
  // Forget everything about the last flight. Called on every disarm, so each
  // arming starts fresh (found by the TLA+ model: DisarmedMeansNoLaunchPending).
  void clearFlight();

  DetectorParams p_;
  FlightStage stage_ = FlightStage::Disarmed;
  bool candidate_ = false;
  float candT_ = 0.0f;
  float accelAboveT_ = 0.0f;
  float baroLaunchT_ = 0.0f;
  bool burnout_ = false;
  float burnoutT_ = 0.0f;
  float launchT_ = -1.0f;
  float apogeeT_ = -1.0f;
  float canopyT_ = -1.0f;
  float maxAgl_ = 0.0f;
  float apogeeCondT_ = 0.0f;
  float lastJoltT_ = -1e9f;
  float lastShockT_ = -1e9f;
  LowPass descent_{0.5f};
  bool streakActive_ = false;
  float streakStart_ = 0.0f;
  float streakMin_ = 0.0f, streakMax_ = 0.0f;
  float landedT_ = 0.0f;
  float lastT_ = 0.0f;
  // Altitude history at 10 Hz for the mean descent rate.
  static constexpr int kHist = 64;  // 6.4 s
  static constexpr float kHistDt = 0.1f;
  float hist_[kHist] = {};
  int histHead_ = 0, histCount_ = 0;
  float histAcc_ = 0.0f;
  float belowMainT_ = -1.0f;       // dual deploy: first time below the main altitude + margin
  float belowMainExactT_ = -1.0f;  // dual deploy: first time below the main altitude itself
  bool mainSeen_ = false;          // the "main fired" sense line has fired this flight
  float gpsBandStart_ = -1.0f;     // when GPS descent entered the canopy band (-1: not in it)
};

}  // namespace gr
