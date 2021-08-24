#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "BasicTypes.hpp"
#include "Utils.hpp"

namespace sp::walk {

namespace inner {

// target position of joints and the profile function on how to get there
// check get_profile to see how the profile functions are used
struct TargetInfo {
  sp::joint_array<double> tgt_pos;
  sp::joint_array<std::function<double(double)>> profile_fn;
};

struct TargetNode {
  TargetInfo targetInfo;
  std::shared_ptr<TargetNode> next;
};

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

joint_array<std::vector<double>> get_incremental_steps(const joint_array<double>& init_pos, const joint_array<double>& tgt_pos, size_t num_steps,
                                                       const joint_array<std::function<double(double)>>& profile_fn) {
  joint_array<std::vector<double>> result;
  for (size_t i = 0; i < init_pos.size(); ++i) {
    result[i] = get_profile(init_pos[i], tgt_pos[i], num_steps, profile_fn[i]);
  }
  return result;
}

void move_decomposed(const TargetInfo& tgt_info, double duration_sec, Spot& spot) {
  const size_t steps_to_tgt = static_cast<size_t>(duration_sec * 1000 / TIME_STEP_MS + 0.5);

  // Get init positions and incremental steps
  joint_array<double> init_pos;
  for (size_t m = 0; m < spot.leg_motors.size(); ++m) init_pos[m] = spot.leg_motors[m]->getTargetPosition();

  joint_array<std::vector<double>> incr_pos = get_incremental_steps(init_pos, tgt_info.tgt_pos, steps_to_tgt, tgt_info.profile_fn);

  // Increment positions until they reach target
  for (size_t s = 0; s < steps_to_tgt; ++s) {
    for (size_t m = 0; m < spot.leg_motors.size(); ++m) {
      spot.leg_motors[m]->setPosition(incr_pos[m][s]);
    }
    spot.step();
  }
}

}  // namespace inner

double fn_linear(double x) { return x; }
double fn_square(double x) { return x * x; }
double fn_cube(double x) { return x * x * x; }

// sigmoid, y approaching 0 and 1 at x = 0 and 1
template <int STEEP>
double fn_tanh(double x) {
  return 0.5 * (tanh((STEEP * x) - static_cast<double>(STEEP) / 2.0)) + 0.5;
}

auto fn_tanh4 = fn_tanh<4>;
auto fn_tanh7 = fn_tanh<7>;

void test_position(double duration_sec, Spot& spot) {
  inner::TargetInfo tgt_info;
  tgt_info.tgt_pos = {0, 0, 0,   // Front left leg
                      0, 0, 0,   // Front right leg
                      0, 0, 0,   // Rear left leg
                      0, 0, 0};  // Rear right leg

  std::vector<std::function<double(double)>> funcs{fn_linear, fn_square, fn_cube, fn_tanh4, fn_tanh7};
  auto pick = [funcs]() { return funcs[abs(static_cast<int>(sp::util::get_random(static_cast<int>(funcs.size()-1))))]; };
  tgt_info.profile_fn = {pick(), pick(), pick(),   // Front left leg
                         pick(), pick(), pick(),   // Front right leg
                         pick(), pick(), pick(),   // Rear left leg
                         pick(), pick(), pick()};  // Rear right leg

  inner::move_decomposed(tgt_info, duration_sec, spot);
}

void test_position2(double duration_sec, Spot& spot) {
  inner::TargetInfo tgt_info;
  tgt_info.tgt_pos = {0, 0, -0.45,  // Front left leg
                      0, 0, -0.45,  // Front right leg
                      0, 0, 1.6,    // Rear left leg
                      0, 0, 1.6};   // Rear right leg
  tgt_info.profile_fn = {fn_cube, fn_linear, fn_linear,  // Front left leg
                         fn_cube, fn_linear, fn_cube,    // Front right leg
                         fn_cube, fn_linear, fn_linear,  // Rear left leg
                         fn_cube, fn_linear, fn_tanh4};  // Rear right leg

  inner::move_decomposed(tgt_info, duration_sec, spot);
}

}  // namespace sp::walk