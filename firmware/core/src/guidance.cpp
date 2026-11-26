#include "gr/guidance.hpp"

#include <cmath>

namespace gr {

float groundSpeedAlong(const Vec2& u, const Vec2& wind, float airspeed) {
  const float wu = wind.dot(u);
  const float cross2 = wind.norm2() - wu * wu;
  const float disc = airspeed * airspeed - cross2;
  if (disc <= 0.0f) return 0.0f;
  return std::fmax(0.0f, wu + std::sqrt(disc));
}

void Guidance::reset() {
  mode_ = GuideMode::None;
  orbitDir_ = 1;
  weaveT_ = 0.0f;
  holding_ = false;
  turnSign_ = 1.0f;
}

float Guidance::heightNeeded(const NavState& n, float brake) const {
  const float d = n.pos.norm();
  if (d < 0.5f) return 0.0f;
  const Vec2 u = -n.pos / d;
  const float va = n.airspeed * (1.0f - c_.brakeSpeedLoss * brake);
  const float sink = n.sinkRate * (1.0f + c_.brakeSinkGain * brake * brake);
  const Vec2 w = n.windValid ? n.wind : Vec2{};
  const float gs = groundSpeedAlong(u, w, va);
  if (gs < 0.3f) return 1e4f;
  return sink * d / gs;
}

bool Guidance::strongWind(const NavState& n) const {
  return n.windValid && n.wind.norm() > g_.strongWindRatio * n.airspeed;
}

float Guidance::crabHeading(float track, const NavState& n, float va) const {
  const Vec2 u = unitFromBearing(track);
  const Vec2 right{-u.y, u.x};
  const float wr = n.wind.dot(right);
  const Vec2 into = -n.wind;
  if (std::fabs(wr) >= 0.95f * va) return bearingOf(into);
  const float h = track + std::asin(-wr / va);
  const float gs = va * std::cos(h - track) + n.wind.dot(u);
  if (gs < 0.5f && into.norm() > 0.5f) return bearingOf(into);  // cannot progress: hold into wind
  return h;
}

float Guidance::steerTrack(float track, const NavState& n) {
  float err;
  if (n.headingValid && n.windValid) {
    err = wrapPi(crabHeading(track, n, n.airspeed) - n.heading);
  } else if (n.courseValid) {
    err = wrapPi(track - n.course);
  } else {
    return 0.0f;
  }
  // Near 180 degrees the sign of the error flips back and forth as the canopy
  // turns; commit to the direction already chosen instead of chattering.
  if (std::fabs(err) > 2.6f) {
    if ((err > 0.0f ? 1.0f : -1.0f) != turnSign_) err = turnSign_ * std::fabs(err);
  } else if (std::fabs(err) > 0.05f) {
    turnSign_ = err > 0.0f ? 1.0f : -1.0f;
  }
  return clampf(g_.courseGain * err, -g_.maxTurnRateCmd, g_.maxTurnRateCmd);
}

float Guidance::finalBrakeFor(const NavState& n, bool& tooHigh, float& excess) const {
  const float h = n.agl;
  const float h0 = heightNeeded(n, 0.0f);
  tooHigh = false;
  if (h <= h0) { excess = h - h0; return 0.0f; }
  const float hMax = heightNeeded(n, g_.finalBrakeMax);
  if (h >= hMax) { tooHigh = true; excess = h - hMax; return g_.finalBrakeMax; }
  excess = 0.0f;
  float lo = 0.0f, hi = g_.finalBrakeMax;
  for (int i = 0; i < 14; ++i) {
    const float mid = 0.5f * (lo + hi);
    if (heightNeeded(n, mid) < h) lo = mid; else hi = mid;
  }
  return 0.5f * (lo + hi);
}

GuidanceCmd Guidance::update(const NavState& n, float dt) {
  GuidanceCmd cmd;
  if (mode_ == GuideMode::None) mode_ = GuideMode::Homing;
  if (!n.posValid) {
    cmd.mode = mode_;
    return cmd;
  }

  const float d = n.pos.norm();
  const float bearingHome = bearingOf(-n.pos);
  const float hNom = heightNeeded(n, g_.finalBrakeNominal);
  cmd.heightNeeded = hNom;

  if (n.agl < g_.flareAlt) mode_ = GuideMode::Flare;

  switch (mode_) {
    case GuideMode::None:
    case GuideMode::Homing: {
      cmd.courseCmd = bearingHome;
      cmd.turnRate = steerTrack(bearingHome, n);
      cmd.symBrake = 0.0f;
      if (n.agl <= hNom) {
        mode_ = GuideMode::Final;
      } else if (d < g_.loiterRadius + g_.loiterEnterMargin && !strongWind(n)) {
        if (n.agl > hNom + g_.loiterExcessHeight) {
          mode_ = GuideMode::Loiter;
          orbitDir_ = (n.pos.cross(n.velGround) >= 0.0f) ? 1 : -1;
        } else {
          mode_ = GuideMode::Final;
        }
      }
      break;
    }

    case GuideMode::Loiter: {
      const float R = g_.loiterRadius;
      const float gamma = bearingOf(n.pos);
      const float lam = static_cast<float>(orbitDir_);
      const float chi = gamma + lam * (kPi / 2.0f + std::atan(g_.orbitGain * (d - R) / R));
      cmd.courseCmd = wrapPi(chi);
      float ff = (std::fabs(d - R) < 0.5f * R) ? lam * n.groundSpeed / R : 0.0f;
      cmd.turnRate = clampf(steerTrack(cmd.courseCmd, n) + ff, -g_.maxTurnRateCmd, g_.maxTurnRateCmd);
      cmd.symBrake = 0.0f;
      if (strongWind(n)) { mode_ = GuideMode::Homing; break; }  // orbit would just drift
      // Turn final when the straight glide home (plus the turn onto it)
      // uses the height we have left.
      const float turnAngle = n.courseValid ? std::fabs(wrapPi(bearingHome - n.course)) : 0.0f;
      const float turnMargin = n.sinkRate * turnAngle / g_.turnRateForMargin;
      if (n.agl <= hNom + turnMargin) {
        mode_ = GuideMode::Final;
        weaveT_ = 0.0f;
      }
      break;
    }

    case GuideMode::Final: {
      bool tooHigh = false;
      float excess = 0.0f;
      cmd.symBrake = finalBrakeFor(n, tooHigh, excess);
      float courseCmd = bearingHome;
      if (d < g_.holdCourseRadius) {
        if (!holding_) { holding_ = true; holdCourse_ = bearingHome; }
        courseCmd = holdCourse_;
      } else if (d > 2.0f * g_.holdCourseRadius) {
        holding_ = false;
      } else if (holding_) {
        courseCmd = holdCourse_;
      }
      if (tooHigh && excess > g_.finalReloiterExcess && d > g_.holdCourseRadius) {
        mode_ = GuideMode::Loiter;
        orbitDir_ = (n.pos.cross(n.velGround) >= 0.0f) ? 1 : -1;
      } else if (tooHigh && excess > g_.weaveExcess && !holding_) {
        weaveT_ += dt;
        const int leg = static_cast<int>(weaveT_ / g_.weavePeriod);
        courseCmd += (leg % 2 == 0 ? 1.0f : -1.0f) * g_.weaveAngle;
      } else {
        weaveT_ = 0.0f;
      }
      cmd.courseCmd = wrapPi(courseCmd);
      cmd.turnRate = steerTrack(cmd.courseCmd, n);
      break;
    }

    case GuideMode::Flare:
      cmd.turnRate = 0.0f;
      cmd.symBrake = g_.flareBrake;
      cmd.courseCmd = n.course;
      break;
  }
  cmd.mode = mode_;
  return cmd;
}

}  // namespace gr
