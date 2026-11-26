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
  const Scenario s_;
  Rng rng_;
  uint64_t frame_ = 0;
  sim::V3 accelBias_, gyroBias_;
  double baroBias_ = 0.0;
  double gpsBN_ = 0.0, gpsBE_ = 0.0, gpsBU_ = 0.0;
  double gpsDropUntil_ = -1.0;  // stress runs: end of the current dropout
  std::deque<PendingFix> pending_;
  gr::GpsFix lastFix_;
  double vbatt_ = 8.1;
};

// Inverse of the firmware's equirectangular projection, for synthesizing fixes.
void localToLatLon(double lat0, double lon0, double north, double east, double& lat, double& lon);

}  // namespace sim
