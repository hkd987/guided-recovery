// Turns truth into what the pod's sensors would report: noise, bias, range
// limits, sample rates, GPS latency and outages.
#pragma once

#include <deque>

#include "gr/types.hpp"
#include "sim/world.hpp"

namespace sim {

class SensorSuite {
 public:
  explicit SensorSuite(const Scenario& s);
  // Fill a 200 Hz input frame from the world's current truth.
  void sample(const World& w, const double duty[2], gr::Inputs& in);
  double batteryV() const { return vbatt_; }

 private:
  struct PendingFix {
    double releaseT;
    gr::GpsFix fix;
  };
  struct Receiver {
    double bN = 0.0, bE = 0.0, bU = 0.0;  // slowly wandering bias
    double dropUntil = -1.0;              // stress runs: end of the current dropout
    std::deque<PendingFix> pending;
  };
  // One GPS receiver's 10 Hz fixes. Receiver 1 uses rng_, exactly as it always has.
  void sampleGps(int idx, Rng& rng, const World& w, double t, gr::GpsFix& out);
  double baroPressure(const World& w, double bias, Rng& rng, int idx);

  const Scenario s_;
  Rng rng_;
  Rng rng2_;  // backup-kit sensors and barometer faults, so nominal draws never change
  uint64_t frame_ = 0;
  sim::V3 accelBias_, gyroBias_;
  double baroBias_ = 0.0, baro2Bias_ = 0.0;
  bool stuckSet_ = false;
  double stuckP_ = 0.0;
  Receiver rx_[2];
  bool rx1DropStarted_ = false;
  gr::GpsFix lastFix_;
  double vbatt_ = 8.1;
};

// Inverse of the firmware's equirectangular projection, for synthesizing fixes.
void localToLatLon(double lat0, double lon0, double north, double east, double& lat, double& lon);

}  // namespace sim
