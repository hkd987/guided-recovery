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

SensorSuite::SensorSuite(const Scenario& s)
    : s_(s), rng_(s.seed * 104729ULL + 3ULL), rng2_(s.seed * 7919ULL + 0x5EC0ULL) {
  accelBias_ = V3{rng_.normal(0, s.accelBias), rng_.normal(0, s.accelBias), rng_.normal(0, s.accelBias)};
  gyroBias_ = V3{rng_.normal(0, s.gyroBias), rng_.normal(0, s.gyroBias), rng_.normal(0, s.gyroBias)};
  baroBias_ = rng_.normal(0.0, 20.0);  // Pa, absolute offset (cancels against the pad reference)
  rx_[0].bN = rng_.normal(0.0, s.gpsBiasSigma);
  rx_[0].bE = rng_.normal(0.0, s.gpsBiasSigma);
  rx_[0].bU = rng_.normal(0.0, 2.0 * s.gpsBiasSigma);
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
    in.baroFresh = true;
    in.pressurePa = static_cast<float>(baroPressure(w, baroBias_, rng_, 0));
    in.baroTempC = 15.0f;
  }
  if (s_.baro2) {
    baro2Bias_ += rng2_.normal(0.0, 0.5 * std::sqrt(dt));
    in.baro2Present = true;
    if (frame_ % 4 == 2) {  // its own 50 Hz, not in step with barometer 1
      in.baroFresh2 = true;
      in.pressurePa2 = static_cast<float>(baroPressure(w, s_.baro2Bias + baro2Bias_, rng2_, 1));
    }
  }

  // ---- GPS, 10 Hz with latency and outages (and a second receiver if fitted).
  sampleGps(0, rng_, w, t, in.gps);
  if (s_.gps2) {
    sampleGps(1, rng2_, w, t, in.gps2);
    in.gps2Present = true;
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

double SensorSuite::baroPressure(const World& w, double bias, Rng& rng, int idx) {
  // Barometer faults hit barometer 1 only.
  double hBias = 0.0;
  if (idx == 0 && s_.fault == FaultType::BaroDrift && w.faultActive())
    hBias = s_.faultMagnitude * (w.t() - w.faultT());
  if (idx == 0 && s_.fault == FaultType::BaroDriftDrogue && w.apogeeT() >= 0.0 && w.t() > w.apogeeT() + 2.0)
    hBias = s_.faultMagnitude * (w.t() - w.apogeeT() - 2.0);
  double p = isaPressure(s_.padElev + w.height() + hBias);
  p += w.pressureTransient();
  p -= 0.01 * w.dynamicPressure();  // static port error at speed
  p += bias + rng.normal(0.0, 2.5);
  if (idx == 0 && s_.fault == FaultType::BaroStuck && w.faultActive()) {
    if (!stuckSet_) { stuckP_ = p; stuckSet_ = true; }
    p = stuckP_;
  }
  return p;
}

void SensorSuite::sampleGps(int idx, Rng& rng, const World& w, double t, gr::GpsFix& out) {
  Receiver& rx = rx_[idx];
  // Under the canopy a stressed scenario degrades the receiver (multipath,
  // the canopy shading the sky). Nominal scenarios use the defaults
  // throughout, with exactly the random draws they always had.
  const bool underCanopy = w.stage() >= TruthStage::Inflating && w.stage() != TruthStage::Landed;
  const bool degraded = underCanopy || !s_.gpsStressCanopyOnly;
  const int rateDiv = degraded ? s_.gpsRateDiv : 20;
  const uint64_t phase = idx == 0 ? 0 : static_cast<uint64_t>(rateDiv / 2);  // receivers not in step
  if ((frame_ + phase) % static_cast<uint64_t>(rateDiv) == 0) {
    const double dtFix = rateDiv == 20 ? 0.1 : rateDiv * 0.005;
    const double a = dtFix / 100.0;  // bias wander, 100 s correlation
    const double k = s_.gpsBiasSigma * std::sqrt(2.0 * a) * (degraded ? s_.gpsBiasScale : 1.0);
    if (idx == 1 && rx.bN == 0.0 && rx.bE == 0.0) {  // receiver 2 starts with its own bias
      rx.bN = rng.normal(0.0, s_.gpsBiasSigma);
      rx.bE = rng.normal(0.0, s_.gpsBiasSigma);
      rx.bU = rng.normal(0.0, 2.0 * s_.gpsBiasSigma);
    }
    rx.bN += -rx.bN * a + k * rng.normal();
    rx.bE += -rx.bE * a + k * rng.normal();
    rx.bU += -rx.bU * a + 2.0 * k * rng.normal();

    bool outage = false;
    if (s_.gpsDropDuringBoost && w.launchT() >= 0.0 && t >= w.launchT() &&
        t < w.burnoutT() + s_.gpsReacquire)
      outage = true;  // both receivers ride in the same airframe
    if (idx == 0 && s_.fault == FaultType::GpsLoss && w.faultActive()) outage = true;  // receiver 1 fails
    if (degraded && s_.gpsDropRate > 0.0) {
      if (t >= rx.dropUntil && rng.uniform(0.0, 1.0) < s_.gpsDropRate * dtFix) {
        rx.dropUntil = t - s_.gpsDropMean * std::log(std::max(1e-9, rng.uniform(0.0, 1.0)));
        if (idx == 0) rx1DropStarted_ = true;
      }
      // Half of receiver 1's dropouts come from the shared sky (the canopy
      // overhead), and hit receiver 2 as well.
      if (idx == 1 && rx1DropStarted_) {
        rx1DropStarted_ = false;
        if (rng.uniform(0.0, 1.0) < 0.5) rx.dropUntil = std::max(rx.dropUntil, rx_[0].dropUntil);
      }
      if (t < rx.dropUntil) outage = true;
    }
    const double pn = degraded ? s_.gpsPosNoise : 0.3;
    const double vn = degraded ? s_.gpsVelNoise : 0.08;
    if (!outage) {
      gr::GpsFix fx;
      const V3 p = w.pos();
      double lat, lon;
      localToLatLon(s_.padLat, s_.padLon, p.x + rx.bN + rng.normal(0, pn), p.y + rx.bE + rng.normal(0, pn), lat,
                    lon);
      fx.fresh = true;
      fx.fixType = 3;
      fx.numSv = static_cast<uint8_t>(degraded ? s_.gpsSvMin + rng.pick(s_.gpsSvRange) : 10 + rng.pick(8));
      fx.latE7 = static_cast<int32_t>(std::llround(lat * 1e7));
      fx.lonE7 = static_cast<int32_t>(std::llround(lon * 1e7));
      fx.altMsl = static_cast<float>(s_.padElev + w.height() + rx.bU + rng.normal(0, (pn / 0.3) * 0.8));
      const V3 v = w.vel();
      fx.velN = static_cast<float>(v.x + rng.normal(0, vn));
      fx.velE = static_cast<float>(v.y + rng.normal(0, vn));
      fx.velD = static_cast<float>(v.z + rng.normal(0, (vn / 0.08) * 0.15));
      fx.hAcc = static_cast<float>((degraded ? s_.gpsHAccBase : 1.2) + std::fabs(rng.normal(0, 0.5)));
      fx.vAcc = fx.hAcc * 1.8f;
      fx.sAcc = static_cast<float>(degraded ? std::max(0.15, 1.8 * vn) : 0.15);
      rx.pending.push_back(PendingFix{t + s_.gpsLatency + (degraded ? s_.gpsLatencyExtra : 0.0), fx});
    }
  }
  while (!rx.pending.empty() && rx.pending.front().releaseT <= t + 1e-9) {
    out = rx.pending.front().fix;
    rx.pending.pop_front();
  }
}

}  // namespace sim
