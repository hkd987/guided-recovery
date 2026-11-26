#include "gr/estimators.hpp"

#include <cmath>

namespace gr {

float isaAltitude(float pressurePa) {
  if (pressurePa <= 0.0f) return 0.0f;
  return 44330.77f * (1.0f - std::pow(pressurePa / 101325.0f, 0.190263f));
}

float pressureToAltitude(float pressurePa, float refPressurePa) {
  // Difference of standard-atmosphere altitudes: uses the lapse rate at the
  // launch site's elevation, not at sea level.
  if (pressurePa <= 0.0f || refPressurePa <= 0.0f) return 0.0f;
  return isaAltitude(pressurePa) - isaAltitude(refPressurePa);
}

// ---------------------------------------------------------------- AltitudeFilter

void AltitudeFilter::reset(float alt) {
  h_ = alt;
  v_ = 0.0f;
  p00_ = square(measNoise) * 4.0f;
  p01_ = 0.0f;
  p11_ = 4.0f;
  rejects_ = 0;
  init_ = true;
}

void AltitudeFilter::predict(float dt) {
  if (!init_ || dt <= 0.0f) return;
  h_ += v_ * dt;
  const float q = square(accelNoise);
  const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
  p00_ += dt * (2.0f * p01_ + dt * p11_) + q * dt4 * 0.25f;
  p01_ += dt * p11_ + q * dt3 * 0.5f;
  p11_ += q * dt2;
}

bool AltitudeFilter::update(float z) {
  if (!std::isfinite(z)) return false;
  if (!init_) { reset(z); return true; }
  const float y = z - h_;
  if (std::fabs(y) > gate && rejects_ < maxRejects) {
    ++rejects_;
    return false;
  }
  if (rejects_ >= maxRejects && std::fabs(y) > gate) {
    // A persistent step: trust the sensor again, keep velocity.
    h_ = z;
    p00_ = square(measNoise) * 4.0f;
    rejects_ = 0;
    return true;
  }
  rejects_ = 0;
  const float s = p00_ + square(measNoise);
  const float k0 = p00_ / s;
  const float k1 = p01_ / s;
  h_ += k0 * y;
  v_ += k1 * y;
  const float p00 = p00_, p01 = p01_;
  p00_ -= k0 * p00;
  p01_ -= k0 * p01;
  p11_ -= k1 * p01;
  return true;
}

// ---------------------------------------------------------------- AttitudeFilter

void AttitudeFilter::reset() {
  init_ = false;
  q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
  bias_ = Vec3{};
}

static void normalizeQuat(float& a, float& b, float& c, float& d) {
  float n = std::sqrt(a * a + b * b + c * c + d * d);
  if (n < 1e-9f) { a = 1.0f; b = c = d = 0.0f; return; }
  a /= n; b /= n; c /= n; d /= n;
}

void AttitudeFilter::update(const Vec3& gyro, const Vec3& accel, float dt, bool useAccel) {
  const float an = accel.norm();
  if (!init_) {
    if (an < 1.0f) return;
    // Align body "up" (-accel direction when still) with NED up.
    // accel at rest = -g * down_body, so down_body = -accel / |accel|.
    Vec3 d = -accel / an;
    // Quaternion rotating body vector d to NED (0,0,1): axis = d x z, angle = acos(d.z)
    Vec3 z{0.0f, 0.0f, 1.0f};
    Vec3 axis = d.cross(z);
    float s = axis.norm();
    float c = d.dot(z);
    if (s < 1e-6f) {
      if (c > 0.0f) { q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f; }
      else { q0_ = 0.0f; q1_ = 1.0f; q2_ = q3_ = 0.0f; }
    } else {
      float angle = std::atan2(s, c);
      axis = axis / s;
      float h = 0.5f * angle;
      q0_ = std::cos(h);
      q1_ = axis.x * std::sin(h);
      q2_ = axis.y * std::sin(h);
      q3_ = axis.z * std::sin(h);
    }
    init_ = true;
    return;
  }
  if (dt <= 0.0f) return;

  Vec3 w = gyro - bias_;
  // Accelerometer correction only when the reading is near 1 g in magnitude.
  if (useAccel && an > 0.6f * kGravity && an < 1.4f * kGravity) {
    Vec3 downMeas = -accel / an;
    Vec3 downEst = downBody();
    Vec3 e = downMeas.cross(downEst);  // rotates estimate toward measurement
    // Mahony: w += kp * e ; bias -= ki * e * dt
    w += e * kp;
    bias_ = bias_ - e * (ki * dt);
    bias_ = Vec3{clampf(bias_.x, -maxBias, maxBias), clampf(bias_.y, -maxBias, maxBias),
                 clampf(bias_.z, -maxBias, maxBias)};
  }

  // Integrate q_dot = 0.5 * q (x) [0, w]
  const float hx = 0.5f * dt * w.x, hy = 0.5f * dt * w.y, hz = 0.5f * dt * w.z;
  float a = q0_, b = q1_, c = q2_, d = q3_;
  q0_ += -b * hx - c * hy - d * hz;
  q1_ += a * hx + c * hz - d * hy;
  q2_ += a * hy - b * hz + d * hx;
  q3_ += a * hz + b * hy - c * hx;
  normalizeQuat(q0_, q1_, q2_, q3_);
}

Vec3 AttitudeFilter::downBody() const {
  // Third row of R(body->NED) = NED z axis expressed in body frame.
  return Vec3{2.0f * (q1_ * q3_ - q0_ * q2_),
              2.0f * (q2_ * q3_ + q0_ * q1_),
              q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_};
}

// ---------------------------------------------------------------- GeoOrigin

static constexpr double kEarthRadius = 6371008.8;
static constexpr double kDeg = 3.14159265358979323846 / 180.0;

void GeoOrigin::set(double latDeg, double lonDeg) {
  lat0 = latDeg;
  lon0 = lonDeg;
  cosLat0 = std::cos(latDeg * kDeg);
  valid = true;
}

Vec2 GeoOrigin::toLocal(double latDeg, double lonDeg) const {
  double dn = (latDeg - lat0) * kDeg * kEarthRadius;
  double de = (lonDeg - lon0) * kDeg * kEarthRadius * cosLat0;
  return Vec2{static_cast<float>(dn), static_cast<float>(de)};
}

// ---------------------------------------------------------------- WindEstimator

void WindEstimator::reset(float defaultAirspeed) {
  head_ = 0;
  count_ = 0;
  valid_ = false;
  wind_ = Vec2{};
  airspeed_ = defaultAirspeed;
  lastCoverageDeg_ = 0.0f;
  lastFitT_ = -1e9f;
}

void WindEstimator::addSample(const Vec2& v, float t, float airspeedNow) {
  // Between circle fits, nudge the wind so that |groundVel - wind| matches
  // the expected airspeed. This tracks head/tailwind changes on straight
  // legs (the crosswind part needs turns, which the next fit supplies).
  if (valid_ && airspeedNow > 0.0f && t - lastFitT_ > 2.0f) {
    const Vec2 air = v - wind_;
    const float an = air.norm();
    if (an > 1.0f) {
      const Vec2 implied = v - air * (airspeedNow / an);
      wind_ += (implied - wind_) * refineGain;
    }
  }
  samples_[head_] = v;
  times_[head_] = t;
  head_ = (head_ + 1) % kCapacity;
  if (count_ < kCapacity) ++count_;
}

void WindEstimator::solve(float t) {
  // Gather recent samples, centred on their mean for numerical conditioning.
  int n = 0;
  double mx = 0.0, my = 0.0;
  for (int i = 0; i < count_; ++i) {
    if (t - times_[i] > maxSampleAge) continue;
    mx += static_cast<double>(samples_[i].x);
    my += static_cast<double>(samples_[i].y);
    ++n;
  }
  if (n < 30) return;
  mx /= n;
  my /= n;

  double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
  for (int i = 0; i < count_; ++i) {
    if (t - times_[i] > maxSampleAge) continue;
    double x = static_cast<double>(samples_[i].x) - mx, y = static_cast<double>(samples_[i].y) - my;
    double z = x * x + y * y;
    sxx += x * x; sxy += x * y; syy += y * y;
    sx += x; sy += y;
    sxz += x * z; syz += y * z; sz += z;
  }
  // Kasa fit: x^2 + y^2 + D x + E y + F = 0
  // [sxx sxy sx][D]   [-sxz]
  // [sxy syy sy][E] = [-syz]
  // [sx  sy  n ][F]   [-sz ]
  double a[3][4] = {{sxx, sxy, sx, -sxz}, {sxy, syy, sy, -syz}, {sx, sy, double(n), -sz}};
  for (int col = 0; col < 3; ++col) {
    int piv = col;
    for (int r = col + 1; r < 3; ++r)
      if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
    if (std::fabs(a[piv][col]) < 1e-9) return;
    if (piv != col)
      for (int k = 0; k < 4; ++k) { double tmp = a[col][k]; a[col][k] = a[piv][k]; a[piv][k] = tmp; }
    for (int r = 0; r < 3; ++r) {
      if (r == col) continue;
      double f = a[r][col] / a[col][col];
      for (int k = col; k < 4; ++k) a[r][k] -= f * a[col][k];
    }
  }
  const double D = a[0][3] / a[0][0];
  const double E = a[1][3] / a[1][1];
  const double F = a[2][3] / a[2][2];
  const double cx = -D / 2.0, cy = -E / 2.0;
  const double r2 = cx * cx + cy * cy - F;
  if (r2 <= 0.0) return;
  const double r = std::sqrt(r2);

  // Residual and angular coverage around the fitted centre.
  bool bins[36] = {};
  double ss = 0.0;
  for (int i = 0; i < count_; ++i) {
    if (t - times_[i] > maxSampleAge) continue;
    double x = static_cast<double>(samples_[i].x) - mx - cx, y = static_cast<double>(samples_[i].y) - my - cy;
    double d = std::sqrt(x * x + y * y) - r;
    ss += d * d;
    double ang = std::atan2(y, x);  // -pi..pi
    int b = static_cast<int>((ang + 3.14159265358979) / (2.0 * 3.14159265358979) * 36.0);
    if (b < 0) b = 0;
    if (b > 35) b = 35;
    bins[b] = true;
  }
  // Coverage = 360 minus the largest run of empty bins (circular).
  int maxGap = 0, run = 0;
  for (int k = 0; k < 72; ++k) {
    if (!bins[k % 36]) { ++run; if (run > maxGap) maxGap = run; }
    else run = 0;
  }
  if (maxGap > 36) maxGap = 36;
  lastCoverageDeg_ = 360.0f - 10.0f * static_cast<float>(maxGap);
  const float rms = static_cast<float>(std::sqrt(ss / n));
  const float rf = static_cast<float>(r);

  const float needCoverage = valid_ ? minCoverageDeg : firstFitCoverageDeg;
  if (lastCoverageDeg_ < needCoverage || rms > maxRmsResidual || rf < minAirspeed || rf > maxAirspeed)
    return;

  Vec2 w{static_cast<float>(cx + mx), static_cast<float>(cy + my)};
  if (valid_) {
    wind_ = wind_ + (w - wind_) * 0.6f;
    airspeed_ += (static_cast<float>(r) - airspeed_) * 0.6f;
  } else {
    wind_ = w;
    airspeed_ = static_cast<float>(r);
  }
  valid_ = true;
  lastFitT_ = t;
}

}  // namespace gr
