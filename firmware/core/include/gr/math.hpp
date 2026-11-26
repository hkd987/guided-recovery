// Small math helpers shared by the flight core. Single precision on purpose:
// the ESP32-S3 has a hardware FPU for float, not double.
#pragma once

#include <cmath>
#include <cstdint>

namespace gr {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kGravity = 9.80665f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float square(float v) { return v * v; }

// Wrap an angle to (-pi, pi].
inline float wrapPi(float a) {
  if (!std::isfinite(a)) return 0.0f;
  a = std::fmod(a + kPi, kTwoPi);
  if (a < 0.0f) a += kTwoPi;
  return a - kPi;
}

// Wrap an angle to [0, 2pi).
inline float wrap2Pi(float a) {
  if (!std::isfinite(a)) return 0.0f;
  a = std::fmod(a, kTwoPi);
  if (a < 0.0f) a += kTwoPi;
  return a;
}

// Horizontal vector in the local frame: x = north, y = east (metres or m/s).
struct Vec2 {
  float x = 0.0f;
  float y = 0.0f;

  Vec2() = default;
  constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(float s) const { return {x * s, y * s}; }
  Vec2 operator/(float s) const { return {x / s, y / s}; }
  Vec2 operator-() const { return {-x, -y}; }
  Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
  Vec2& operator*=(float s) { x *= s; y *= s; return *this; }

  float dot(const Vec2& o) const { return x * o.x + y * o.y; }
  float cross(const Vec2& o) const { return x * o.y - y * o.x; }
  float norm() const { return std::sqrt(x * x + y * y); }
  float norm2() const { return x * x + y * y; }
};

// Unit vector along a bearing measured clockwise from north.
inline Vec2 unitFromBearing(float bearing) { return {std::cos(bearing), std::sin(bearing)}; }
// Bearing (clockwise from north) of a vector.
inline float bearingOf(const Vec2& v) { return std::atan2(v.y, v.x); }

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  Vec3() = default;
  constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
  Vec3 operator-() const { return {-x, -y, -z}; }
  Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }

  float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  float norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3 normalized() const {
    float n = norm();
    return n > 1e-9f ? (*this) / n : Vec3{0.0f, 0.0f, 0.0f};
  }
};

// First-order low-pass filter with a time constant in seconds.
class LowPass {
 public:
  explicit LowPass(float tau = 1.0f) : tau_(tau) {}
  void setTau(float tau) { tau_ = tau; }
  void reset(float v) { y_ = v; init_ = true; }
  float update(float x, float dt) {
    if (!init_) { reset(x); return y_; }
    float a = (tau_ <= 0.0f) ? 1.0f : clampf(dt / (tau_ + dt), 0.0f, 1.0f);
    y_ += a * (x - y_);
    return y_;
  }
  float value() const { return y_; }
  bool initialized() const { return init_; }

 private:
  float tau_;
  float y_ = 0.0f;
  bool init_ = false;
};

}  // namespace gr
