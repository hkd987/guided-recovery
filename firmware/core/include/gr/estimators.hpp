// State estimation: barometric altitude, attitude (for turn rate), local
// position from GPS, and wind/airspeed from GPS ground velocity.
#pragma once

#include <cstdint>

#include "gr/math.hpp"

namespace gr {

// Standard-atmosphere pressure altitude (m above the 1013.25 hPa level).
float isaAltitude(float pressurePa);
// Altitude of pressurePa above the level where refPressurePa was measured.
float pressureToAltitude(float pressurePa, float refPressurePa);

// Two-state (altitude, climb rate) Kalman filter driven by the barometer.
// Rejects single wild samples (ejection-charge pressure spikes) but accepts a
// persistent step after a few consecutive rejections.
class AltitudeFilter {
 public:
  void reset(float alt);
  void predict(float dt);
  // Returns false if the sample was rejected as an outlier.
  bool update(float measuredAlt);
  float altitude() const { return h_; }
  float climbRate() const { return v_; }
  bool initialized() const { return init_; }

  float accelNoise = 4.0f;   // m/s^2 (process)
  float measNoise = 0.35f;   // m (baro)
  float gate = 15.0f;        // m innovation gate
  int maxRejects = 6;

 private:
  bool init_ = false;
  float h_ = 0.0f, v_ = 0.0f;
  float p00_ = 25.0f, p01_ = 0.0f, p11_ = 25.0f;
  int rejects_ = 0;
};

// Mahony complementary filter. We only need the gravity direction in the body
// frame, to turn gyro rates into a turn rate about the vertical. Yaw is left
// free: guidance steers on GPS course, so no magnetometer is required.
class AttitudeFilter {
 public:
  void reset();
  // useAccel = false while the accelerometer is not measuring gravity (boost,
  // coast, tumbling under a drogue); the gyro carries the estimate alone.
  void update(const Vec3& gyro, const Vec3& accel, float dt, bool useAccel = true);
  // Unit "down" vector expressed in the body frame.
  Vec3 downBody() const;
  // Rotation rate about the vertical, rad/s, positive clockwise seen from above.
  float yawRate(const Vec3& gyro) const { return gyro.dot(downBody()); }
  bool initialized() const { return init_; }

  float kp = 0.6f;
  float ki = 0.02f;
  float maxBias = 0.1f;  // rad/s

 private:
  bool init_ = false;
  float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;  // body -> NED
  Vec3 bias_;
};

// Converts GPS lat/lon to metres north/east of an origin. Equirectangular is
// accurate to centimetres over the few kilometres a pod can travel.
struct GeoOrigin {
  bool valid = false;
  double lat0 = 0.0, lon0 = 0.0;  // degrees
  double cosLat0 = 1.0;
  void set(double latDeg, double lonDeg);
  Vec2 toLocal(double latDeg, double lonDeg) const;
};

// Estimates the wind and the canopy's airspeed from GPS ground velocity alone.
// While the canopy turns, ground-velocity samples trace a circle whose centre
// is the wind vector and whose radius is the airspeed. A least-squares circle
// fit over the last ~20 s gives both, with no compass needed.
class WindEstimator {
 public:
  static constexpr int kCapacity = 200;

  void reset(float defaultAirspeed);
  // airspeedNow: the airspeed expected at this moment (trim airspeed reduced
  // for any symmetric brake). Used to refine the wind between turns.
  void addSample(const Vec2& groundVel, float t, float airspeedNow = -1.0f);
  // Fits when enough heading coverage exists. Call about once a second.
  void solve(float t);

  bool valid() const { return valid_; }
  Vec2 wind() const { return wind_; }
  float airspeed() const { return airspeed_; }
  float coverageDeg() const { return lastCoverageDeg_; }
  float lastFitTime() const { return lastFitT_; }

  float minCoverageDeg = 200.0f;
  float firstFitCoverageDeg = 270.0f;
  float refineGain = 0.02f;  // per sample, along-heading wind refinement
  float maxRmsResidual = 0.9f;
  float minAirspeed = 3.0f;
  float maxAirspeed = 16.0f;
  float maxSampleAge = 20.0f;

 private:
  Vec2 samples_[kCapacity];
  float times_[kCapacity] = {};
  int head_ = 0;
  int count_ = 0;
  bool valid_ = false;
  Vec2 wind_;
  float airspeed_ = 8.0f;
  float lastCoverageDeg_ = 0.0f;
  float lastFitT_ = -1e9f;
};

}  // namespace gr
