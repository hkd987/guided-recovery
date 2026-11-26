#include "sim/sensors.hpp"

#include <algorithm>

namespace sim {

namespace {
constexpr double kEarthRadius = 6371008.8;
constexpr double kDeg = kPi / 180.0;
constexpr double kAccelRange = 24.0 * kG;          // BMI088 at +-24 g
constexpr double kGyroRange = 2000.0 * kPi / 180.0;  // 2000 dps

double sat(double v, double lim) { return clampd(v, -lim, lim); }
}  // namespace

void localToLatLon(double lat0, double lon0, double north, double east, double& lat, double& lon) {
  lat = lat0 + north / kEarthRadius / kDeg;
  lon = lon0 + east / (kEarthRadius * std::cos(lat0 * kDeg)) / kDeg;
}

SensorSuite::SensorSuite(const Scenario& s) : s_(s), rng_(s.seed * 104729ULL + 3ULL) {
  accelBias_ = V3{rng_.normal(0, s.accelBias), rng_.normal(0, s.accelBias), rng_.normal(0, s.accelBias)};
  gyroBias_ = V3{rng_.normal(0, s.gyroBias), rng_.normal(0, s.gyroBias), rng_.normal(0, s.gyroBias)};
  baroBias_ = rng_.normal(0.0, 20.0);  // Pa, absolute offset (cancels against the pad reference)
  gpsBN_ = rng_.normal(0.0, s.gpsBiasSigma);
  gpsBE_ = rng_.normal(0.0, s.gpsBiasSigma);
  gpsBU_ = rng_.normal(0.0, 2.0 * s.gpsBiasSigma);
}

void SensorSuite::sample(const World& w, const double duty[2], gr::Inputs& in) {
  const double t = w.t();
  const double dt = 0.005;
  in = gr::Inputs{};
  in.tUs = static_cast<uint64_t>(std::llround(t * 1e6));

  // ---- IMU, every frame.
  V3 f = w.specificForceBody() + accelBias_;
  const double vib = w.stage() == TruthStage::Boost ? 3.0 : 0.05;
  f += V3{rng_.normal(0, vib), rng_.normal(0, vib), rng_.normal(0, vib)};
  V3 gy = w.omegaBody() + gyroBias_ + V3{rng_.normal(0, 0.004), rng_.normal(0, 0.004), rng_.normal(0, 0.004)};
  in.imu.accel = gr::Vec3{static_cast<float>(sat(f.x, kAccelRange)), static_cast<float>(sat(f.y, kAccelRange)),
                          static_cast<float>(sat(f.z, kAccelRange))};
  in.imu.gyro = gr::Vec3{static_cast<float>(sat(gy.x, kGyroRange)), static_cast<float>(sat(gy.y, kGyroRange)),
                         static_cast<float>(sat(gy.z, kGyroRange))};
  in.imu.valid = true;

  // ---- Barometer, 50 Hz.
  baroBias_ += rng_.normal(0.0, 0.5 * std::sqrt(dt));
  if (frame_ % 4 == 0) {
    double p = isaPressure(s_.padElev + w.height());
    p += w.pressureTransient();
    p -= 0.01 * w.dynamicPressure();  // static port error at speed
    p += baroBias_ + rng_.normal(0.0, 2.5);
    in.baroFresh = true;
    in.pressurePa = static_cast<float>(p);
    in.baroTempC = 15.0f;
  }

  // ---- GPS, 10 Hz with latency and outages.
  if (frame_ % 20 == 0) {
    const double a = 0.1 / 100.0;  // bias wander, 100 s correlation
    const double k = s_.gpsBiasSigma * std::sqrt(2.0 * a);
    gpsBN_ += -gpsBN_ * a + k * rng_.normal();
    gpsBE_ += -gpsBE_ * a + k * rng_.normal();
    gpsBU_ += -gpsBU_ * a + 2.0 * k * rng_.normal();

    bool outage = false;
    if (s_.gpsDropDuringBoost && w.launchT() >= 0.0 && t >= w.launchT() &&
        t < w.burnoutT() + s_.gpsReacquire)
      outage = true;
    if (s_.fault == FaultType::GpsLoss && w.faultActive()) outage = true;
    if (!outage) {
      gr::GpsFix fx;
      const V3 p = w.pos();
      double lat, lon;
      localToLatLon(s_.padLat, s_.padLon, p.x + gpsBN_ + rng_.normal(0, 0.3), p.y + gpsBE_ + rng_.normal(0, 0.3),
                    lat, lon);
      fx.fresh = true;
      fx.fixType = 3;
      fx.numSv = static_cast<uint8_t>(10 + rng_.pick(8));
      fx.latE7 = static_cast<int32_t>(std::llround(lat * 1e7));
      fx.lonE7 = static_cast<int32_t>(std::llround(lon * 1e7));
      fx.altMsl = static_cast<float>(s_.padElev + w.height() + gpsBU_ + rng_.normal(0, 0.8));
      const V3 v = w.vel();
      fx.velN = static_cast<float>(v.x + rng_.normal(0, 0.08));
      fx.velE = static_cast<float>(v.y + rng_.normal(0, 0.08));
      fx.velD = static_cast<float>(v.z + rng_.normal(0, 0.15));
      fx.hAcc = static_cast<float>(1.2 + std::fabs(rng_.normal(0, 0.5)));
      fx.vAcc = fx.hAcc * 1.8f;
      fx.sAcc = 0.15f;
      pending_.push_back(PendingFix{t + s_.gpsLatency, fx});
    }
  }
  while (!pending_.empty() && pending_.front().releaseT <= t + 1e-9) {
    in.gps = pending_.front().fix;
    pending_.pop_front();
  }

  // ---- Encoders, switches, battery.
  in.encoder[0] = w.winch(0).counts(s_);
  in.encoder[1] = w.winch(1).counts(s_);
  in.armed = t >= s_.armTime;
  in.mainFiredInput = false;
  vbatt_ = 8.15 - 0.0003 * t - 0.25 * (std::fabs(duty[0]) + std::fabs(duty[1]));
  in.batteryV = static_cast<float>(vbatt_);
  ++frame_;
}

}  // namespace sim
