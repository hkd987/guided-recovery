// Recognises the host rocket's flight events with no wiring to its flight
// computer: launch, burnout, apogee, and (the important one) "canopy open".
//
// Canopy confirmation follows the spec:
//   1. ignore boost and coast,
//   2. ignore drogue descent (fast fall that never slows to parafoil speed),
//   3. look for an opening jolt, then a steady descent in the parafoil band,
//   4. only then may steering arm.
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
};

class PhaseDetector {
 public:
  explicit PhaseDetector(const DetectorParams& p) : p_(p) {}
  void reset();
  void update(const DetectorInput& in, float dt);

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
  float steadyFor() const { return streakActive_ ? (lastT_ - streakStart_) : 0.0f; }
  float smoothedDescent() const { return descent_.value(); }

 private:
  void resetStreak(float t, float d);
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
  LowPass descent_{0.5f};
  bool streakActive_ = false;
  float streakStart_ = 0.0f;
  float streakMin_ = 0.0f, streakMax_ = 0.0f;
  float landedT_ = 0.0f;
  float lastT_ = 0.0f;
};

}  // namespace gr
