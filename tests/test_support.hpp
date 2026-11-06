#pragma once
#include <cmath>
#include <iostream>
#include <stdexcept>
#define CHECK(expr)                                                                                  \
  do {                                                                                               \
    if (!(expr))                                                                                     \
      throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #expr); \
  } while (false)
template <class F>
void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::exception&) {
    caught = true;
  }
  CHECK(caught);
}
inline bool near(double a, double b, double tol = 1e-12) { return std::fabs(a - b) <= tol; }
