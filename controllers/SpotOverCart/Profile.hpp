#pragma once

#include <vector>
#include <functional>

namespace sp {
namespace fn {
double linear(double x) { return x; }
double square(double x) { return x * x; }
double square_inv(double x) { return 1 - pow(x - 1, 2); }  // assumes we only use 0 to 1
double cube(double x) { return x * x * x; }
double cube_inv(double x) { return pow(x - 1, 3) + 1; }  // assumes we only use 0 to 1

// sigmoid, y approaching 0 and 1 at x = 0 and 1
template <int STEEP>
double tan_h(double x) {
  return 0.5 * (tanh((STEEP * x) - static_cast<double>(STEEP) / 2.0)) + 0.5;
}

auto tanh4 = tan_h<4>;
auto tanh7 = tan_h<7>;
}  // namespace fn

template <typename T>
std::vector<T> get_profile(T fr, T to, size_t num_steps, std::function<T(T)> fn) {
  static_assert(std::is_floating_point<T>::value == true, "range must be in floating point");

  std::vector<T> profile;
  bool inverted = false;
  if (to <= fr) {
    inverted = true;
    std::swap(to, fr);
  }

  // get base profile 0 to 1
  for (size_t i = 0; i < num_steps; ++i) {
    T x = static_cast<T>(i + 1) / static_cast<T>(num_steps);
    profile.push_back(fn(x));
  }

  // scale to range
  T range = to - fr;
  double max = static_cast<double>(profile.back());
  for (size_t i = 0; i < profile.size(); ++i) profile[i] = fr + range * profile[i] / max;

  if (inverted) {
    std::swap(fr, to);  // for readability
    for (size_t i = 0; i < profile.size(); ++i) profile[i] = fr - (profile[i] - to);
  }
  return profile;
}
}