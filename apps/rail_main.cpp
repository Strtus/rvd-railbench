#include "rvd/runtime.hpp"
#include <cstdio>
#include <stdexcept>
#if defined(_WIN32) && defined(RVD_WINDOWS_TIMER)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <timeapi.h>
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
