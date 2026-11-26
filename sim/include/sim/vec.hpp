// Double-precision vectors and rotations for the simulator's truth model.
#pragma once

#include <cmath>
#include <cstdint>
#include <random>

namespace sim {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double wrapPi(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0) a += 2.0 * kPi;
  return a - kPi;
}

struct V3 {
  double x = 0, y = 0, z = 0;
  V3() = default;
  constexpr V3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  V3 operator*(double s) const { return {x * s, y * s, z * s}; }
  V3 operator/(double s) const { return {x / s, y / s, z / s}; }
  V3 operator-() const { return {-x, -y, -z}; }
  V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  double dot(const V3& o) const { return x * o.x + y * o.y + z * o.z; }
  V3 cross(const V3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  V3 unit() const { double n = norm(); return n > 1e-12 ? *this / n : V3{0, 0, 1}; }
};

// Rotation matrix, body -> NED (columns are body axes expressed in NED).
struct M3 {
  double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  V3 col(int c) const { return {m[0][c], m[1][c], m[2][c]}; }
  V3 operator*(const V3& v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }
  V3 tmul(const V3& v) const {  // transpose * v  (NED -> body)
    return {m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z, m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
            m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z};
  }
  M3 operator*(const M3& o) const {
    M3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        r.m[i][j] = 0;
        for (int k = 0; k < 3; ++k) r.m[i][j] += m[i][k] * o.m[k][j];
      }
    return r;
  }
  M3 transpose() const {
    M3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
    return r;
  }
  static M3 fromColumns(const V3& a, const V3& b, const V3& c) {
    M3 r;
    r.m[0][0] = a.x; r.m[1][0] = a.y; r.m[2][0] = a.z;
    r.m[0][1] = b.x; r.m[1][1] = b.y; r.m[2][1] = b.z;
    r.m[0][2] = c.x; r.m[1][2] = c.y; r.m[2][2] = c.z;
    return r;
  }
  // ZYX Euler: R = Rz(yaw) Ry(pitch) Rx(roll)
  static M3 euler(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch),
                 cy = std::cos(yaw), sy = std::sin(yaw);
    M3 r;
    r.m[0][0] = cy * cp; r.m[0][1] = cy * sp * sr - sy * cr; r.m[0][2] = cy * sp * cr + sy * sr;
    r.m[1][0] = sy * cp; r.m[1][1] = sy * sp * sr + cy * cr; r.m[1][2] = sy * sp * cr - cy * sr;
    r.m[2][0] = -sp;     r.m[2][1] = cp * sr;                r.m[2][2] = cp * cr;
    return r;
  }
  // Integrate by a body-frame rotation vector (small angle).
  M3 rotatedBody(const V3& w, double dt) const {
    const V3 a = w * dt;
    const double th = a.norm();
    M3 d;
    if (th > 1e-12) {
      const V3 k = a / th;
      const double c = std::cos(th), s = std::sin(th), v = 1 - c;
      d.m[0][0] = c + k.x * k.x * v;       d.m[0][1] = k.x * k.y * v - k.z * s; d.m[0][2] = k.x * k.z * v + k.y * s;
      d.m[1][0] = k.y * k.x * v + k.z * s; d.m[1][1] = c + k.y * k.y * v;       d.m[1][2] = k.y * k.z * v - k.x * s;
      d.m[2][0] = k.z * k.x * v - k.y * s; d.m[2][1] = k.z * k.y * v + k.x * s; d.m[2][2] = c + k.z * k.z * v;
    }
    return (*this) * d;
  }
};

// Body rates that take attitude a to attitude b in dt.
inline V3 bodyRates(const M3& a, const M3& b, double dt) {
  const M3 d = a.transpose() * b;
  return V3{(d.m[2][1] - d.m[1][2]) * 0.5 / dt, (d.m[0][2] - d.m[2][0]) * 0.5 / dt, (d.m[1][0] - d.m[0][1]) * 0.5 / dt};
}

class Rng {
 public:
  explicit Rng(uint64_t seed) : g_(seed) {}
  double uniform(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(g_); }
  double normal(double mean = 0.0, double sd = 1.0) { return std::normal_distribution<double>(mean, sd)(g_); }
  bool chance(double p) { return uniform(0.0, 1.0) < p; }
  int pick(int n) { return std::uniform_int_distribution<int>(0, n - 1)(g_); }
  uint64_t next() { return g_(); }

 private:
  std::mt19937_64 g_;
};

// ISA pressure (Pa) at geometric altitude above mean sea level.
inline double isaPressure(double altMsl) { return 101325.0 * std::pow(1.0 - 2.25577e-5 * altMsl, 5.25588); }
inline double isaDensity(double altMsl) { return 1.225 * std::pow(1.0 - 2.25577e-5 * altMsl, 4.25588); }

}  // namespace sim
