#include "gr/phase_detector.hpp"

#include <cmath>

namespace gr {

void PhaseDetector::reset() {
  stage_ = FlightStage::Disarmed;
  candidate_ = false;
  candT_ = accelAboveT_ = baroLaunchT_ = 0.0f;
  burnout_ = false;
  burnoutT_ = 0.0f;
  launchT_ = apogeeT_ = canopyT_ = -1.0f;
  maxAgl_ = 0.0f;
  apogeeCondT_ = 0.0f;
  lastJoltT_ = -1e9f;
  descent_ = LowPass(0.5f);
  streakActive_ = false;
  landedT_ = 0.0f;
  lastT_ = 0.0f;
  histHead_ = histCount_ = 0;
  histAcc_ = 0.0f;
  belowMainT_ = -1.0f;
  belowMainExactT_ = -1.0f;
}

void PhaseDetector::pushAltitude(float agl, float dt) {
  histAcc_ += dt;
  if (histCount_ > 0 && histAcc_ < kHistDt) return;
  histAcc_ = histCount_ > 0 ? histAcc_ - kHistDt : 0.0f;
  if (histAcc_ > kHistDt) histAcc_ = 0.0f;  // a long gap: don't try to catch up
  hist_[histHead_] = agl;
  histHead_ = (histHead_ + 1) % kHist;
  if (histCount_ < kHist) ++histCount_;
}

bool PhaseDetector::meanDescent(float& rate) const {
  int n = static_cast<int>(p_.steadyWindow / kHistDt + 0.5f);
  if (n < 1) n = 1;
  if (n > kHist - 1) n = kHist - 1;
  if (histCount_ < n + 1) return false;
  const float now = hist_[(histHead_ + kHist - 1) % kHist];
  const float then = hist_[(histHead_ + kHist - 1 - n) % kHist];
  rate = (then - now) / (static_cast<float>(n) * kHistDt);
  return true;
}

void PhaseDetector::resumeAt(FlightStage stage) {
  reset();
  stage_ = stage;
  burnout_ = true;  // a restart mid-boost is not worth modelling; thrust is long over
}

void PhaseDetector::clearFlight() {
  candidate_ = false;
  candT_ = accelAboveT_ = baroLaunchT_ = 0.0f;
  burnout_ = false;
  burnoutT_ = 0.0f;
  launchT_ = apogeeT_ = canopyT_ = -1.0f;
  maxAgl_ = 0.0f;
  apogeeCondT_ = 0.0f;
  lastJoltT_ = -1e9f;
  streakActive_ = false;
  landedT_ = 0.0f;
  histHead_ = histCount_ = 0;
  histAcc_ = 0.0f;
  belowMainT_ = -1.0f;
  belowMainExactT_ = -1.0f;
}

void PhaseDetector::resetStreak(float t, float d) {
  streakActive_ = true;
  streakStart_ = t;
  streakMin_ = streakMax_ = d;
}

void PhaseDetector::update(const DetectorInput& in, float dt) {
  lastT_ = in.t;
  descent_.update(-in.climbRate, dt);

  switch (stage_) {
    case FlightStage::Disarmed:
      if (in.armed) stage_ = FlightStage::Pad;
      break;

    case FlightStage::Pad: {
      if (!in.armed) { stage_ = FlightStage::Disarmed; clearFlight(); break; }
      // Accelerometer path: sustained high g, confirmed by the barometer.
      if (in.accelG > p_.launchAccelG) accelAboveT_ += dt;
      else accelAboveT_ = 0.0f;
      if (!candidate_ && accelAboveT_ >= p_.launchAccelTime) {
        candidate_ = true;
        candT_ = in.t - accelAboveT_;
      }
      if (candidate_) {
        if (in.baroValid && in.agl > p_.launchConfirmAlt) {
          stage_ = FlightStage::Ascent;
          launchT_ = candT_;
        } else if (in.t - candT_ > p_.launchConfirmWindow) {
          candidate_ = false;  // handling bump, not a launch
          accelAboveT_ = 0.0f;
        }
      }
      // Barometer-only path, in case the accelerometer saturated or failed.
      if (stage_ == FlightStage::Pad && in.baroValid && in.agl > p_.baroLaunchAlt &&
          in.climbRate > p_.baroLaunchClimb) {
        baroLaunchT_ += dt;
        if (baroLaunchT_ > 0.5f) {
          stage_ = FlightStage::Ascent;
          launchT_ = in.t - in.agl / std::fmax(in.climbRate, 1.0f);
        }
      } else {
        baroLaunchT_ = 0.0f;
      }
      if (stage_ == FlightStage::Ascent) maxAgl_ = in.agl;
      break;
    }

    case FlightStage::Ascent:
    case FlightStage::Descent:
    case FlightStage::Canopy: {
      // Burnout: thrust gone. Jolts only count after this, so motor thrust
      // can never be mistaken for an opening shock.
      if (!burnout_) {
        if (in.accelG < p_.burnoutAccelG) {
          burnoutT_ += dt;
          if (burnoutT_ >= p_.burnoutTime) burnout_ = true;
        } else {
          burnoutT_ = 0.0f;
        }
      } else if (in.accelG > p_.joltAccelG) {
        lastJoltT_ = in.t;
      }
      if (in.mainFired) lastJoltT_ = in.t;

      if (stage_ == FlightStage::Ascent) {
        if (in.agl > maxAgl_) maxAgl_ = in.agl;
        bool falling = in.climbRate < p_.apogeeClimb;
        apogeeCondT_ = falling ? apogeeCondT_ + dt : 0.0f;
        if (apogeeCondT_ >= p_.apogeeTime || in.agl < maxAgl_ - p_.apogeeDrop) {
          stage_ = FlightStage::Descent;
          apogeeT_ = in.t;
          streakActive_ = false;
          histHead_ = histCount_ = 0;  // the mean descent starts at apogee
          histAcc_ = 0.0f;
        }
        break;
      }

      if (stage_ == FlightStage::Descent) {
        if (in.baroValid) pushAltitude(in.agl, dt);
        float mean = 0.0f;
        const bool haveMean = in.baroValid && meanDescent(mean);
        // Dual deploy (mainAlt set): nothing above the main altitude can be
        // the main. The steady streak only runs below it, and only a jolt
        // below it (the main coming out) counts, so a slow drogue that
        // happens to fall at canopy speed is not mistaken for the canopy.
        const bool dual = p_.mainAlt > 0.0f;
        const bool belowMain = !dual || in.agl <= p_.mainAlt + std::fmax(15.0f, 0.1f * p_.mainAlt);
        if (belowMain && belowMainT_ < 0.0f) belowMainT_ = in.t;
        if (dual && in.agl <= p_.mainAlt && belowMainExactT_ < 0.0f) belowMainExactT_ = in.t;
        const bool inBand = haveMean && belowMain && mean >= p_.bandLow && mean <= p_.bandHigh;
        if (!inBand) {
          streakActive_ = false;
        } else if (!streakActive_) {
          resetStreak(in.t, mean);
        } else {
          if (mean < streakMin_) streakMin_ = mean;
          if (mean > streakMax_) streakMax_ = mean;
          if (streakMax_ - streakMin_ > p_.steadySpread) resetStreak(in.t, mean);
        }
        if (streakActive_) {
          const float steady = in.t - streakStart_;
          bool jolted = (in.t - lastJoltT_) <= p_.joltWindow + steady;
          if (dual) jolted = jolted && belowMainT_ >= 0.0f && lastJoltT_ >= belowMainT_;
          // Without a jolt there is no sign the main came out, so in dual
          // deploy the no-jolt clock only runs below the main altitude itself
          // (a slow drogue crossing the margin above it must not count).
          float steadyNoJolt = steady;
          if (dual) steadyNoJolt = belowMainExactT_ >= 0.0f ? in.t - std::fmax(streakStart_, belowMainExactT_) : 0.0f;
          if ((jolted && steady >= p_.confirmTime) || steadyNoJolt >= p_.confirmTimeNoJolt) {
            stage_ = FlightStage::Canopy;
            canopyT_ = in.t;
          }
        }
      }

      // Landed: no vertical motion for a while, near the ground.
      if (stage_ >= FlightStage::Descent) {
        bool still = std::fabs(in.climbRate) < p_.landedClimb && in.agl < p_.landedMaxAlt;
        landedT_ = still ? landedT_ + dt : 0.0f;
        if (landedT_ >= p_.landedTime) stage_ = FlightStage::Landed;
      }
      break;
    }

    case FlightStage::Landed:
      if (!in.armed) { stage_ = FlightStage::Disarmed; clearFlight(); }
      break;
  }
}

}  // namespace gr
