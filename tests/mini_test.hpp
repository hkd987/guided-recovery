// A deliberately tiny test harness so the repo builds with no downloads.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace mt {

struct Case {
  const char* name;
  std::function<void()> fn;
};

inline std::vector<Case>& registry() {
  static std::vector<Case> r;
  return r;
}
inline int& failures() {
  static int f = 0;
  return f;
}

struct Registrar {
  Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

}  // namespace mt

#define MT_CAT2(a, b) a##b
#define MT_CAT(a, b) MT_CAT2(a, b)
#define TEST(name)                                                   \
  static void name();                                                \
  static mt::Registrar MT_CAT(reg_, name)(#name, name);              \
  static void name()

#define CHECK(cond)                                                                   \
  do {                                                                                \
    if (!(cond)) {                                                                    \
      std::printf("  FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);                 \
      ++mt::failures();                                                               \
    }                                                                                 \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                  \
  do {                                                                                         \
    double va_ = (a), vb_ = (b);                                                               \
    if (!(std::fabs(va_ - vb_) <= (tol))) {                                                    \
      std::printf("  FAILED %s:%d: %s = %g, expected %g +- %g\n", __FILE__, __LINE__, #a, va_, \
                  vb_, static_cast<double>(tol));                                              \
      ++mt::failures();                                                                        \
    }                                                                                          \
  } while (0)
