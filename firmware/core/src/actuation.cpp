#include "gr/actuation.hpp"

#include <cmath>

namespace gr {

BrakeCmd mixBrakes(float sym, float diff) {
  sym = clampf(sym, 0.0f, 1.0f);
  diff = clampf(diff, -1.0f, 1.0f);
  float right = sym + (diff > 0.0f ? diff : 0.0f);
  float left = sym + (diff < 0.0f ? -diff : 0.0f);
  // Keep the turn: if one side would pass full travel, ease the other off.
  if (right > 1.0f) { left -= right - 1.0f; right = 1.0f; }
  if (left > 1.0f) { right -= left - 1.0f; left = 1.0f; }
  return BrakeCmd{clampf(left, 0.0f, 1.0f), clampf(right, 0.0f, 1.0f)};
}

void Steering::reset() {
  rate_ = LowPass(s_.yawRateTau);
  diffLag_ = LowPass(0.6f);
  diffLag_.reset(0.0f);
  diffFast_ = LowPass(0.1f);
  diffFast_.reset(0.0f);
  th0_ = c_.turnRatePerBrake > 0.05f ? c_.turnRatePerBrake : 0.9f;
  th1_ = 0.0f;
  p00_ = 0.06f; p01_ = 0.0f; p11_ = 0.09f;
  accD_ = accR_ = accT_ = 0.0f;
  accN_ = 0;
  integ_ = 0.0f;
  sinceReset_ = 0.0f;
  lopsidedT_ = 0.0f;
  stuckT_ = 0.0f;
  eff_[0] = eff_[1] = 0.0f;
  heldT_[0] = heldT_[1] = 0.0f;
  pinT_ = 0.0f;
  pinDir_ = 0.0f;
  lopsided_ = false;
}

BrakeCmd Steering::update(float rCmd, float sym, float yawRateMeas, float actualDiff, float dt) {
  const float r = rate_.update(yawRateMeas, dt);
  const float err = rCmd - r;
  const float kTurn = c_.turnRatePerBrake > 0.05f ? c_.turnRatePerBrake : 0.05f;
  float diff = rCmd / kTurn + s_.kp * err + integ_;
  const bool saturated = std::fabs(diff) >= 1.0f;
  // Anti-windup: only integrate when not saturated, or when it would unwind.
  if (!saturated || (diff > 0.0f) != (err > 0.0f)) {
    integ_ = clampf(integ_ + s_.ki * err * dt, -s_.trimLimit, s_.trimLimit);
  }
  diff = clampf(rCmd / kTurn + s_.kp * err + integ_, -1.0f, 1.0f);

  // Learn turnRate = K * diff + bias with a 2-state Kalman filter at 20 Hz
  // (averaged samples), random-walk parameters: the bias may change within
  // seconds (a line snags), the gain only slowly.
  sinceReset_ += dt;
  const float d = diffLag_.update(actualDiff, dt);
  const float dFast = diffFast_.update(actualDiff, dt);
  // Learn only from quasi-steady moments: mid-reversal, the canopy's own lag
  // (which differs between canopies) would masquerade as a bias.
  const bool steady = std::fabs(dFast - d) < 0.12f;
  if (!steady) { accD_ = accR_ = accT_ = 0.0f; accN_ = 0; }
  accD_ += d;
  accR_ += r;
  accT_ += dt;
  ++accN_;
  if (sinceReset_ < 0.75f * s_.lopsidedSettle) {
    // Let the swing after opening die down first. Canopy confirmation can
    // now come within ~3 s of the opening (mean-descent test), so wait 4.5 s.
    accD_ = accR_ = accT_ = 0.0f;
    accN_ = 0;
  } else if (accT_ >= 0.05f) {
    const float dm = accD_ / accN_, rm = accR_ / accN_, h = accT_;
    accD_ = accR_ = accT_ = 0.0f;
    accN_ = 0;
    p00_ += 1e-4f * h;   // gain drift (0.01 rad/s per sqrt s)
    p11_ += 2.5e-3f * h; // bias drift (0.05 rad/s per sqrt s)
    const float R = 0.01f;  // (0.1 rad/s)^2: swing, gusts, model error
    const float q0 = p00_ * dm + p01_, q1 = p01_ * dm + p11_;
    const float den = dm * q0 + q1 + R;
    const float k0 = q0 / den, k1 = q1 / den;
    // Clamp the innovation: one wild sample must not drag the estimate.
    const float sd = std::sqrt(den);
    const float e = clampf(rm - (th0_ * dm + th1_), -2.5f * sd, 2.5f * sd);
    th0_ = clampf(th0_ + k0 * e, 0.25f, 2.0f);
    th1_ = clampf(th1_ + k1 * e, -1.5f, 1.5f);
    p00_ -= k0 * q0;
    p01_ -= k0 * q1;
    p11_ -= k1 * q1;
    p00_ = clampf(p00_, 1e-5f, 0.06f);
    p11_ = clampf(p11_, 1e-5f, 0.09f);
    const float lim = 0.99f * std::sqrt(p00_ * p11_);
    p01_ = clampf(p01_, -lim, lim);
  }
  const float needed = trimNeeded();
  const bool confident = trimNeededSd() < s_.lopsidedMaxSd;
  if (sinceReset_ > s_.lopsidedSettle && confident && std::fabs(needed) > s_.lopsidedTrim) lopsidedT_ += dt;
  else lopsidedT_ = std::fmax(0.0f, lopsidedT_ - dt);
  if (lopsidedT_ >= s_.lopsidedTime) lopsided_ = true;

  // Stuck: brake pinned toward the commanded turn, turn not happening.
  // Each side's effectiveness (turn per unit of brake, with the brake held
  // past 40 % for 2 s so the canopy has caught up) is remembered, so a canopy
  // that is merely weak, equally weak both ways, is not called lopsided.
  const float fade = std::exp(-dt / 60.0f);
  for (int i = 0; i < 2; ++i) eff_[i] *= fade;
  for (int i = 0; i < 2; ++i) {
    const float sgn = i == 0 ? 1.0f : -1.0f;
    if (actualDiff * sgn >= 0.4f) heldT_[i] += dt;
    else heldT_[i] = 0.0f;
    if (heldT_[i] > 2.0f && sinceReset_ > 2.0f)
      eff_[i] = std::fmax(eff_[i], (r * sgn) / std::fabs(actualDiff));
  }
  const float dir = actualDiff >= s_.stuckDiff ? 1.0f : (actualDiff <= -s_.stuckDiff ? -1.0f : 0.0f);
  // Judge only a brake held hard over for a while: weaving swaps sides every
  // couple of seconds and the canopy's lag would read as "not turning".
  pinT_ = (dir != 0.0f && dir == pinDir_) ? pinT_ + dt : 0.0f;
  pinDir_ = dir;
  bool stuck = false;
  if (dir != 0.0f && pinT_ >= 2.0f && sinceReset_ > s_.lopsidedSettle) {
    const int side = dir > 0.0f ? 0 : 1, other = 1 - side;
    const float ach = r * dir;                         // turn achieved toward the pinned side
    const float effHere = ach / std::fabs(actualDiff);
    const bool otherKnown = eff_[other] > 0.0f;
    const bool otherMuchBetter = otherKnown && eff_[other] >= 2.0f * std::fmax(effHere, 0.05f);
    const bool otherAsWeak = otherKnown && !otherMuchBetter;
    const float cmd = rCmd * dir;
    if (cmd >= s_.stuckMinCmd) {
      // Barely turns (unless the other side is just as weak), or clearly
      // turns far better the other way.
      stuck = (ach < s_.stuckTurnFrac * cmd && !otherAsWeak) || (ach < 0.6f * cmd && otherMuchBetter);
    }
  }
  if (stuck) stuckT_ += dt;
  else stuckT_ = std::fmax(0.0f, stuckT_ - 0.5f * dt);
  if (stuckT_ >= s_.stuckTime) lopsided_ = true;

  return mixBrakes(sym, diff);
}

float Steering::trimNeededSd() const {
  // trim = -b/K; first-order propagation of the parameter covariance.
  const float t = trimNeeded();
  const float v = p11_ + 2.0f * t * p01_ + t * t * p00_;
  return std::sqrt(std::fmax(v, 0.0f)) / th0_;
}

void WinchController::configure(const WinchParams& p, int channel) {
  p_ = p;
  ch_ = channel;
  reset();
}

void WinchController::reset() {
  init_ = false;
  posMm_ = lastPosMm_ = targetMm_ = 0.0f;
  lastBaseMm_ = 0.0f;
  lastMovePosMm_ = 0.0f;
  idleT_ = 0.0f;
  exerciseT_ = -1.0f;
  vel_ = LowPass(0.03f);
  stallT_ = 0.0f;
  wrongT_ = 0.0f;
  stalled_ = false;
  wrongWay_ = false;
}

float WinchController::update(float frac, int32_t counts, float batteryV, float dt, bool powered) {
  const float dir = p_.direction[ch_] < 0 ? -1.0f : 1.0f;
  posMm_ = dir * static_cast<float>(counts) / p_.countsPerMm;
  if (!init_) { lastPosMm_ = posMm_; init_ = true; }
  const float v = dt > 0.0f ? vel_.update((posMm_ - lastPosMm_) / dt, dt) : vel_.value();
  lastPosMm_ = posMm_;

  const float base = clampf(frac, 0.0f, 1.0f) * p_.travelMm;
  if (!powered) {
    targetMm_ = lastBaseMm_ = base;
    lastMovePosMm_ = posMm_;
    idleT_ = 0.0f;
    exerciseT_ = -1.0f;
    stallT_ = 0.0f;
    return 0.0f;
  }
  // Periodic small exercise of an idle winch, so a jam shows up early.
  // "Idle" means the spool hasn't actually moved: a spool jammed near zero
  // never moves, however often small corrections change its target.
  if (std::fabs(posMm_ - lastMovePosMm_) > 2.0f) { lastMovePosMm_ = posMm_; idleT_ = 0.0f; }
  else idleT_ += dt;
  // A real steering move takes priority over an exercise in progress.
  if (std::fabs(base - lastBaseMm_) > 10.0f) { exerciseT_ = -1.0f; lastBaseMm_ = base; }
  float offset = 0.0f;
  if (exerciseT_ < 0.0f && idleT_ >= p_.exerciseIdle) exerciseT_ = 0.0f;
  if (exerciseT_ >= 0.0f) {
    exerciseT_ += dt;
    offset = (base < p_.travelMm - 2.0f * p_.exerciseMm) ? p_.exerciseMm : -p_.exerciseMm;
    if (exerciseT_ >= p_.exerciseTime) { exerciseT_ = -1.0f; idleT_ = 0.0f; offset = 0.0f; }
  }
  targetMm_ = base + offset;
  const float err = targetMm_ - posMm_;
  float duty = 0.0f;
  if (std::fabs(err) > p_.deadbandMm) duty = p_.kp * err - p_.kd * v;
  const float vmax = batteryV > 1.0f ? clampf(p_.motorMaxVolts / batteryV, 0.0f, 1.0f) : 1.0f;
  duty = clampf(duty, -vmax, vmax);

  // Stall: pushing hard, not moving, still far from target.
  if (std::fabs(duty) >= p_.stallDuty * vmax && std::fabs(v) < p_.stallSpeedMmS &&
      std::fabs(err) > p_.stallMinErrMm) {
    stallT_ += dt;
    if (stallT_ >= p_.stallTime) stalled_ = true;
  } else {
    // Leaky: several short pushes against a jam add up.
    stallT_ = std::fmax(0.0f, stallT_ - 0.25f * dt);
  }
  // Wrong way: pushing one way, moving fast the other way (miswired encoder).
  if (std::fabs(duty) >= 0.3f * vmax && v * duty < 0.0f && std::fabs(v) > 5.0f) {
    wrongT_ += dt;
    if (wrongT_ >= 0.2f) { wrongWay_ = true; stalled_ = true; }
  } else {
    wrongT_ = std::fmax(0.0f, wrongT_ - dt);
  }
  if (stalled_) duty = 0.0f;  // stop cooking a jammed motor
  return dir * duty;
}

}  // namespace gr
