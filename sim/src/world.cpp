#include "sim/world.hpp"

#include <algorithm>

namespace sim {

const char* truthStageName(TruthStage s) {
  switch (s) {
    case TruthStage::Pad: return "pad";
    case TruthStage::Boost: return "boost";
    case TruthStage::Coast: return "coast";
    case TruthStage::Drogue: return "drogue";
    case TruthStage::Inflating: return "inflating";
    case TruthStage::Canopy: return "canopy";
    case TruthStage::Landed: return "landed";
  }
  return "?";
}

// ------------------------------------------------------------------ wind

void WindField::init(const Scenario& s, uint64_t seed) {
  s_ = &s;
  rng_ = Rng(seed ^ 0xA5A5A5A5ULL);
  gN_ = rng_.normal(0.0, s.gustSigma);
  gE_ = rng_.normal(0.0, s.gustSigma);
  rngV_ = Rng(seed ^ 0x5A5A5A5AULL);
  gD_ = s.vertGustSigma > 0.0 ? rngV_.normal(0.0, s.vertGustSigma) : 0.0;
  thermalPhase_ = rngV_.uniform(0.0, 2.0 * kPi);
}

void WindField::step(double dt) {
  const double a = dt / s_->gustTau;
  const double k = s_->gustSigma * std::sqrt(2.0 * a);
  gN_ += -gN_ * a + k * rng_.normal();
  gE_ += -gE_ * a + k * rng_.normal();
  if (s_->vertGustSigma > 0.0) {
    const double av = dt / s_->vertGustTau;
    gD_ += -gD_ * av + s_->vertGustSigma * std::sqrt(2.0 * av) * rngV_.normal();
  }
}

double WindField::verticalAt(double h, double t) const {
  // Down-positive (NED). Vertical air dies out toward the ground.
  if (s_->vertGustSigma <= 0.0 && s_->thermalAmp <= 0.0) return 0.0;
  const double fade = clampd(h / 30.0, 0.0, 1.0);
  double up = -gD_;
  if (s_->thermalAmp > 0.0) {
    const double c = std::sin(2.0 * kPi * t / s_->thermalPeriod + thermalPhase_);
    up += s_->thermalAmp * (c > 0.0 ? c * c : 0.0);
  }
  return -up * fade;
}

V3 WindField::steadyAt(double h) const {
  const Scenario& s = *s_;
  const double hh = std::max(h, 1.0);
  double speed = s.windRef * std::pow(hh / 10.0, s.shearExp);
  speed = std::min(speed, s.windRef * 2.2);
  double from = s.windFrom + s.veer * clampd(h / 300.0, 0.0, 1.5);
  if (s.fault == FaultType::Shear && h < 60.0) {
    const double f = 1.0 - std::max(h, 0.0) / 60.0;
    from += s.faultMagnitude * f;
    speed *= 1.0 + 0.4 * f;
  }
  const double toward = from + kPi;
  return V3{speed * std::cos(toward), speed * std::sin(toward), 0.0};
}

V3 WindField::at(double h, double t) const {
  V3 w = steadyAt(h);
  // Gusts shrink near the ground with the mean wind.
  w += V3{gN_, gE_, 0.0};
  if (s_->fault == FaultType::Gust && t >= faultTime_ && t < faultTime_ + 6.0) {
    const double toward = s_->windFrom + kPi;
    const double m = s_->faultMagnitude * std::sin(kPi * (t - faultTime_) / 6.0);
    w += V3{m * std::cos(toward), m * std::sin(toward), 0.0};
  }
  return w;
}

// ------------------------------------------------------------------ winch

void WinchTruth::step(double duty, double vbatt, double loadN, double dt, bool jammed, const Scenario& s) {
  if (jammed) { speed = 0.0; return; }
  const double v = clampd(duty, -1.0, 1.0) * vbatt * 0.95;  // H-bridge drop
  double target;
  if (std::fabs(v) < 0.25) {
    // Unpowered: the gearbox holds unless line tension back-drives it.
    target = loadN > s.winchBackdriveN ? -(loadN - s.winchBackdriveN) * 15.0 : 0.0;
  } else {
    // DC motor speed-torque line, scaled by voltage. Line tension always
    // resists pulling in and helps letting out.
    // DC motor: speed falls linearly with load torque, slope independent of voltage.
    const double vScale = v / 6.0;
    target = s.winchNoLoadMmS * vScale - s.winchNoLoadMmS * (loadN / s.winchStallN);
    if (v > 0.0 && target < 0.0) target = 0.0;  // stalled, not reversing
  }
  speed += (target - speed) * clampd(dt / 0.03, 0.0, 1.0);
  pos += speed * dt;
  pos = clampd(pos, -40.0, 160.0);
  if (pos <= -40.0 || pos >= 160.0) speed = 0.0;
}

// ------------------------------------------------------------------ world

World::World(const Scenario& s) : s_(s), rng_(s.seed * 7919ULL + 17ULL) {
  wind_.init(s_, s.seed);
  const double tilt = s_.railTilt;
  railDir_ = V3{std::sin(tilt) * std::cos(s_.railAz), std::sin(tilt) * std::sin(s_.railAz), -std::cos(tilt)};
  nose_ = railDir_;
  rocketAttitude(nose_);
  swingP1_ = rng_.uniform(0.0, 2.0 * kPi);
  swingP2_ = rng_.uniform(0.0, 2.0 * kPi);
  fBody_ = R_.tmul(V3{0.0, 0.0, -kG});
}

double World::brakeFrac(int ch) const {
  return clampd(winch_[ch].pos / s_.brakeTravelMm, 0.0, 1.0);
}

void World::overrideBrakes(double fracL, double fracR) {
  overridden_ = true;
  winch_[0].pos = clampd(fracL, 0.0, 1.0) * s_.brakeTravelMm;
  winch_[1].pos = clampd(fracR, 0.0, 1.0) * s_.brakeTravelMm;
}

void World::rocketAttitude(const V3& nose) {
  // Body z points out the tail; body x is the most northerly perpendicular.
  const V3 z = -nose.unit();
  V3 ref{1.0, 0.0, 0.0};
  if (std::fabs(z.dot(ref)) > 0.95) ref = V3{0.0, 1.0, 0.0};
  const V3 x = (ref - z * z.dot(ref)).unit();
  const V3 y = z.cross(x);
  R_ = M3::fromColumns(x, y, z);
}

void World::ejectionEvent(double shockG, double pressurePa) {
  spikeEndT_ = t_ + 0.008;
  spikeVec_ = V3{rng_.normal(), rng_.normal(), rng_.normal()}.unit() * (shockG * kG);
  lastChargeT_ = t_;
  lastChargePa_ = pressurePa;
}

void World::deployParafoil() {
  stage_ = TruthStage::Inflating;
  deployT_ = t_;
  deployH_ = height();
  deployPos_ = pos_;
  psi_ = rng_.uniform(-kPi, kPi);
  r_ = 0.0;
  ejectionEvent(s_.ejectShockG, s_.ejectPressurePa);
}

void World::step(double dt, const double duty[2], double vbatt) {
  t_ += dt;
  wind_.step(dt);
  const M3 Rprev = R_;
  const V3 velPrev = vel_;
  const double h = height();
  const V3 w = wind_.at(h, t_);
  const V3 g{0.0, 0.0, kG};

  // Winches (always simulated; loads only exist under an open canopy).
  if (!overridden_) {
    for (int ch = 0; ch < 2; ++ch) {
      const double load = stage_ == TruthStage::Canopy ? 0.3 + s_.lineForceFull * brakeFrac(ch) : 0.05;
      const bool jam = s_.fault == FaultType::WinchStall && faultActive() && ch == s_.faultSide;
      winch_[ch].step(duty[ch], vbatt, load, dt, jam, s_);
    }
  }

  bool pendulum = false;
  double swingAmp = 0.0;
  switch (stage_) {
    case TruthStage::Pad:
      if (t_ >= s_.launchTime) {
        stage_ = TruthStage::Boost;
        launchT_ = t_;
      }
      break;

    case TruthStage::Boost:
    case TruthStage::Coast: {
      const double tb = t_ - launchT_;
      const double ramp = std::min(1.0, tb / 0.05);
      const double thrust = tb < s_.burnTime ? s_.thrust * ramp : 0.0;
      const double mass = s_.mass * (1.0 - s_.propFrac * std::min(tb / s_.burnTime, 1.0));
      const V3 vAir = vel_ - w;
      const double va = vAir.norm();
      const double rho = isaDensity(s_.padElev + h);
      const double area = kPi * s_.diameter * s_.diameter / 4.0;
      qbar_ = 0.5 * rho * va * va;
      const double drag = qbar_ * s_.cd * area;
      const bool onRail = pos_.norm() < s_.railLength && stage_ == TruthStage::Boost;
      if (!onRail && va > 3.0) nose_ = vAir.unit();
      else if (onRail) nose_ = railDir_;
      V3 a = nose_ * (thrust / mass) - (va > 1e-6 ? vAir / va : V3{}) * (drag / mass) + g;
      if (onRail) a = railDir_ * std::max(0.0, a.dot(railDir_));
      vel_ += a * dt;
      pos_ += vel_ * dt;
      rocketAttitude(nose_);
      if (stage_ == TruthStage::Boost && tb >= s_.burnTime) stage_ = TruthStage::Coast;
      if (height() > maxH_) maxH_ = height();
      if (apogeeT_ < 0.0 && stage_ == TruthStage::Coast && vel_.z > 0.0) apogeeT_ = t_;

      // Recovery events.
      if (s_.dualDeploy) {
        if (apogeeT_ >= 0.0 && t_ >= apogeeT_ + 0.5) {
          if (height() > s_.mainAlt + 30.0) {
            stage_ = TruthStage::Drogue;
            drogueOut_ = true;
            ejectionEvent(s_.ejectShockG, s_.ejectPressurePa * 0.5);
          } else {
            deployParafoil();
          }
        }
      } else if (s_.ejectErr >= 0.0) {
        if (apogeeT_ >= 0.0 && t_ >= apogeeT_ + s_.ejectErr) deployParafoil();
      } else if (stage_ == TruthStage::Coast && -vel_.z < -s_.ejectErr * kG) {
        deployParafoil();  // early ejection, still climbing
      }
      break;
    }

    case TruthStage::Drogue: {
      qbar_ = 0.0;
      const V3 vAir = vel_ - w;
      const double va = vAir.norm();
      const double k = kG / (s_.drogueRate * s_.drogueRate);
      const V3 a = g - vAir * (k * va);
      vel_ += a * dt;
      pos_ += vel_ * dt;
      // Airframe tumbling and swinging under the drogue.
      const V3 wb{2.0 * std::sin(1.3 * t_ + swingP1_), 1.5 * std::cos(0.9 * t_ + swingP2_), 0.8};
      R_ = R_.rotatedBody(wb, dt);
      if (height() <= s_.mainAlt && !s_.mainFailure) deployParafoil();
      break;
    }

    case TruthStage::Inflating: {
      qbar_ = 0.0;
      const double ti = t_ - deployT_;
      const V3 vss = V3{s_.Va0 * std::cos(psi_) + w.x, s_.Va0 * std::sin(psi_) + w.y,
                        s_.vs0 + wind_.verticalAt(-pos_.z, t_)};
      vel_ += (vss - vel_) * clampd(dt / (s_.inflationTime / 3.0), 0.0, 1.0);
      pos_ += vel_ * dt;
      pendulum = true;
      swingAmp = s_.swingAmp * 1.5;
      if (ti >= s_.inflationTime) {
        stage_ = TruthStage::Canopy;
        openT_ = t_;
        faultT_ = openT_ + s_.faultAfterOpen;
        if (s_.fault == FaultType::Shear) faultT_ = openT_;  // atmosphere, always there
        wind_.setFaultTime(faultT_);
      }
      break;
    }

    case TruthStage::Canopy: {
      qbar_ = 0.0;
      const double dL = brakeFrac(0), dR = brakeFrac(1);
      const double da = dR - dL;
      const double ds = std::min(dL, dR);
      const double va = s_.Va0 * (1.0 - s_.brakeSpeedLoss * ds);
      double bias = s_.turnBias;
      if (s_.fault == FaultType::Lopsided && faultActive()) bias += s_.faultMagnitude;
      const double rss = s_.Kturn * da * (va / s_.Va0) + bias;
      r_ += (rss - r_) * clampd(dt / s_.turnTau, 0.0, 1.0);
      psi_ = wrapPi(psi_ + r_ * dt);
      const double bank = std::atan(va * r_ / kG);
      const double vs = s_.vs0 * (1.0 + s_.brakeSinkGain * ds * ds) / std::cos(bank);
      const V3 vt{va * std::cos(psi_) + w.x, va * std::sin(psi_) + w.y, vs + wind_.verticalAt(-pos_.z, t_)};
      vel_ += (vt - vel_) * clampd(dt / 0.4, 0.0, 1.0);
      pos_ += vel_ * dt;
      pendulum = true;
      swingAmp = s_.swingAmp * std::exp(-(t_ - openT_) / 4.0) + s_.swingResidual;
      break;
    }

    case TruthStage::Landed:
      vel_ = V3{};
      qbar_ = 0.0;
      break;
  }

  acc_ = (vel_ - velPrev) / dt;
  const V3 fNed = acc_ - g;

  if (pendulum) {
    // The pod hangs along the apparent gravity, plus a decaying swing. A
    // pendulum's accelerometer reads along its line, so it cannot see the
    // swing tilt; only the gyro can.
    const double fmag = fNed.norm();
    const double c = std::cos(psi_), sn = std::sin(psi_);
    const double aFwd = acc_.x * c + acc_.y * sn;
    const double aRight = -acc_.x * sn + acc_.y * c;
    const double phi = std::asin(clampd(aRight / fmag, -0.95, 0.95));
    const double theta = std::asin(clampd(-aFwd / (fmag * std::cos(phi)), -0.95, 0.95));
    const double om = 2.0 * kPi / 1.6;
    const double rollS = 0.6 * swingAmp * std::sin(om * t_ + swingP1_);
    const double pitchS = swingAmp * std::sin(om * t_ + swingP2_);
    double twist = 0.0;  // harness twist: the pod yaws back and forth under the canopy
    if (s_.yawWobbleAmp > 0.0 && stage_ == TruthStage::Canopy) {
      const double ow = 2.0 * kPi / s_.yawWobblePeriod;
      twist = (s_.yawWobbleAmp / ow) * std::sin(ow * t_ + swingP1_ + swingP2_);
    }
    R_ = M3::euler(phi + rollS, theta + pitchS, psi_ + twist);
    fBody_ = V3{0.0, 0.0, -fmag};
  } else if (stage_ == TruthStage::Drogue) {
    fBody_ = R_.tmul(fNed) + V3{rng_.normal(0, 2.0), rng_.normal(0, 2.0), rng_.normal(0, 2.0)};
  } else if (stage_ == TruthStage::Landed) {
    fBody_ = R_.tmul(V3{0.0, 0.0, -kG});
  } else {
    fBody_ = R_.tmul(fNed);
  }

  // Shock spikes: ejection charges, and the canopy snatching open.
  if (t_ < spikeEndT_) fBody_ += spikeVec_;
  if (stage_ == TruthStage::Inflating) {
    const double ts = t_ - (deployT_ + 0.4 * s_.inflationTime);
    if (ts >= 0.0 && ts < 0.15) fBody_.z -= s_.openShockG * kG * std::sin(kPi * ts / 0.15);
  }
  omega_ = bodyRates(Rprev, R_, dt);

  // Ejection gas pressurizes the bay for a moment.
  const double tc = t_ - lastChargeT_;
  pressureExtra_ = (tc >= 0.0 && tc < 0.6) ? lastChargePa_ * std::exp(-tc / 0.12) : 0.0;

  // Touchdown.
  if (stage_ != TruthStage::Pad && stage_ != TruthStage::Landed && stage_ != TruthStage::Boost && pos_.z >= 0.0) {
    stage_ = TruthStage::Landed;
    landT_ = t_;
    pos_.z = 0.0;
    landPos_ = pos_;
    vel_ = V3{};
    rocketAttitude(V3{0.0, 0.0, -1.0});
  }
}

}  // namespace sim
