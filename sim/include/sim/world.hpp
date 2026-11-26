// Truth model: rocket ascent, recovery events, parafoil flight, wind, and the
// two winch motors. Everything the sensors sample comes from here.
#pragma once

#include "sim/scenario.hpp"
#include "sim/vec.hpp"

namespace sim {

enum class TruthStage { Pad = 0, Boost, Coast, Drogue, Inflating, Canopy, Landed };
const char* truthStageName(TruthStage s);

class WindField {
 public:
  void init(const Scenario& s, uint64_t seed);
  void step(double dt);
  // Wind velocity (NED, z = 0) at height h above the pad, time t.
  V3 at(double h, double t) const;
  // Steady part only (no gusts), used by the "perfect knowledge" gate-1 runner.
  V3 steadyAt(double h) const;
  void setFaultTime(double t) { faultTime_ = t; }

 private:
  const Scenario* s_ = nullptr;
  Rng rng_{1};
  double gN_ = 0, gE_ = 0;
  double faultTime_ = 1e18;
};

struct WinchTruth {
  double pos = 0.0;    // mm of line pulled in (0 = brakes released)
  double speed = 0.0;  // mm/s
  void step(double duty, double vbatt, double loadN, double dt, bool jammed, const Scenario& s);
  int32_t counts(const Scenario& s) const { return static_cast<int32_t>(std::lround(pos * s.winchCountsPerMm)); }
};

class World {
 public:
  explicit World(const Scenario& s);

  // Advance by dt with the given motor duties (-1..1).
  void step(double dt, const double duty[2], double vbatt);
  // Gate 1 only: set brake positions directly (ideal actuators).
  void overrideBrakes(double fracL, double fracR);

  const Scenario& scenario() const { return s_; }
  double t() const { return t_; }
  TruthStage stage() const { return stage_; }
  double height() const { return -pos_.z; }
  const V3& pos() const { return pos_; }
  const V3& vel() const { return vel_; }
  const M3& attitude() const { return R_; }
  const V3& omegaBody() const { return omega_; }
  const V3& specificForceBody() const { return fBody_; }
  double pressureTransient() const { return pressureExtra_; }
  double dynamicPressure() const { return qbar_; }
  double heading() const { return psi_; }
  double turnRate() const { return r_; }
  double brakeFrac(int ch) const;
  const WinchTruth& winch(int ch) const { return winch_[ch]; }
  V3 windHere() const { return wind_.at(height(), t_); }
  V3 steadyWindHere() const { return wind_.steadyAt(height()); }

  double launchT() const { return launchT_; }
  double burnoutT() const { return launchT_ >= 0 ? launchT_ + s_.burnTime : -1.0; }
  double apogeeT() const { return apogeeT_; }
  double apogeeH() const { return maxH_; }
  double deployT() const { return deployT_; }
  double deployH() const { return deployH_; }
  V3 deployPos() const { return deployPos_; }
  double openT() const { return openT_; }
  double landT() const { return landT_; }
  V3 landPos() const { return landPos_; }
  double faultT() const { return faultT_; }
  bool faultActive() const { return t_ >= faultT_; }

 private:
  void deployParafoil();
  void ejectionEvent(double shockG, double pressurePa);
  void rocketAttitude(const V3& nose);

  Scenario s_;
  Rng rng_;
  WindField wind_;
  double t_ = 0.0;
  TruthStage stage_ = TruthStage::Pad;
  V3 pos_, vel_, acc_;
  V3 railDir_;
  V3 nose_;
  M3 R_;
  V3 omega_, fBody_;
  double qbar_ = 0.0;
  double pressureExtra_ = 0.0;
  double lastChargeT_ = -1e9, lastChargePa_ = 0.0;
  double spikeEndT_ = -1.0;
  V3 spikeVec_;
  double psi_ = 0.0, r_ = 0.0;
  double swingP1_ = 0.0, swingP2_ = 0.0;
  WinchTruth winch_[2];
  bool overridden_ = false;
  double launchT_ = -1, apogeeT_ = -1, maxH_ = 0, deployT_ = -1, deployH_ = 0, openT_ = -1, landT_ = -1;
  double faultT_ = 1e18;
  V3 deployPos_, landPos_;
  bool drogueOut_ = false;
};

}  // namespace sim
