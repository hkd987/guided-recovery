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
}

void PhaseDetector::resetStreak(float t, float d) {
  streakActive_ = true;
  streakStart_ = t;
  streakMin_ = streakMax_ = d;
}

void PhaseDetector::update(const DetectorInput& in, float dt) {
  lastT_ = in.t;
  const float descent = descent_.update(-in.climbRate, dt);

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
        }
        break;
      }

      if (stage_ == FlightStage::Descent) {
        const bool inBand = descent >= p_.bandLow && descent <= p_.bandHigh;
        if (!inBand) {
          streakActive_ = false;
        } else if (!streakActive_) {
          resetStreak(in.t, descent);
        } else {
          if (descent < streakMin_) streakMin_ = descent;
          if (descent > streakMax_) streakMax_ = descent;
          if (streakMax_ - streakMin_ > p_.steadySpread) resetStreak(in.t, descent);
        }
        if (streakActive_) {
          const float steady = in.t - streakStart_;
          const bool jolted = (in.t - lastJoltT_) <= p_.joltWindow + steady;
          if ((jolted && steady >= p_.confirmTime) || steady >= p_.confirmTimeNoJolt) {
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
