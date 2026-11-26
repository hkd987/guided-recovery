#include <cstdio>
#include <cstring>

#include "mini_test.hpp"

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int ran = 0;
  for (const auto& c : mt::registry()) {
    if (filter && !std::strstr(c.name, filter)) continue;
    const int before = mt::failures();
    c.fn();
    std::printf("%s %s\n", mt::failures() == before ? "ok  " : "FAIL", c.name);
    ++ran;
  }
  std::printf("%d tests, %d failed checks\n", ran, mt::failures());
  return mt::failures() == 0 ? 0 : 1;
}
