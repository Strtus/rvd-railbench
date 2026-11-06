#include <cstdio>
#include <stdexcept>

#include "rvd/runtime.hpp"
#if defined(_WIN32) && defined(RVD_WINDOWS_TIMER)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
// clang-format off: Windows types must precede the multimedia declarations.
#include <windows.h>
#include <timeapi.h>
// clang-format on
struct TimerResolution {
  TimerResolution() { timeBeginPeriod(1); }
  ~TimerResolution() { timeEndPeriod(1); }
};
#endif
int main(int argc, char** argv) {
#if defined(_WIN32) && defined(RVD_WINDOWS_TIMER)
  TimerResolution timer;
#endif
  try {
    const auto opt = rvd::parse_options(argc, argv);
    return rvd::run(opt);
  } catch (const std::exception& e) {
    if (std::string(e.what()) == "help") return 0;
    std::fprintf(stderr, "fatal: %s\n", e.what());
    return 2;
  }
}
