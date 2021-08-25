#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "BasicTypes.hpp"
#include "Utils.hpp"

namespace sp::walk {

namespace fn {
double linear(double x) { return x; }
double square(double x) { return x * x; }
double cube(double x) { return x * x * x; }

// sigmoid, y approaching 0 and 1 at x = 0 and 1
template <int STEEP>
double tan_h(double x) {
  return 0.5 * (tanh((STEEP * x) - static_cast<double>(STEEP) / 2.0)) + 0.5;
}

auto tanh4 = tan_h<4>;
auto tanh7 = tan_h<7>;
}  // namespace fn

namespace inner {
// target position of joints and the profile function on how to get there
// check get_profile to see how the profile functions are used
struct TargetInfo {
  TargetInfo() {
    tgt_pos.fill(0);
    profile_fn.fill(fn::linear);
  }
  sp::joint_array<double> tgt_pos{0};
  sp::joint_array<std::function<double(double)>> profile_fn{fn::linear};
};

// todo:
//   add conditional to go to next, some kind of state management
//     check during steps to go early
struct TargetNode {
  TargetInfo tgt_info;
  std::shared_ptr<TargetNode> next{nullptr};
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

void chain_nodes(std::shared_ptr<TargetNode> node, double duration_sec, Spot& spot) {
  std::shared_ptr<TargetNode> curr = node;
  while (curr != nullptr) {
    std::cout << "[SPOT] .... going to next node " << curr->tgt_info.tgt_pos << std::endl;
    move_decomposed(curr->tgt_info, duration_sec, spot);
    curr = curr->next;
  }
  std::cout << "[SPOT] ... finished" << std::endl;
}

}  // namespace inner

void test_position(double duration_sec, Spot& spot) {
  inner::TargetInfo tgt_info;

  std::vector<std::function<double(double)>> funcs{fn::linear, fn::square, fn::cube, fn::tanh4, fn::tanh7};
  auto pick = [funcs]() { return funcs[abs(static_cast<int>(sp::util::get_random(static_cast<int>(funcs.size() - 1))))]; };
  tgt_info.profile_fn = {pick(), pick(), pick(),   // Front left leg
                         pick(), pick(), pick(),   // Front right leg
                         pick(), pick(), pick(),   // Rear left leg
                         pick(), pick(), pick()};  // Rear right leg

  inner::move_decomposed(tgt_info, duration_sec, spot);
}

void spread_legs(double duration_sec, Spot& spot) {
  inner::TargetInfo tgt_info;
  const double abduct = 0.2;
  tgt_info.tgt_pos = {-abduct, 0, 0,   // Front left leg
                      abduct,  0, 0,   // Front right leg
                      -abduct, 0, 0,   // Rear left leg
                      abduct,  0, 0};  // Rear right leg
  inner::move_decomposed(tgt_info, duration_sec, spot);
}

void walk_version1(double duration_sec, Spot& spot) {
  const double shoulder1 = -0.3;
  const double shoulder2 = 0.4;
  const double elbow1 = 0.3;
  const double elbow2 = -0.2;
  const double abduct = 0.2;

  auto elbow_mv = fn::square;
  auto shoulder_mv = fn::tanh4;

  // clang-format off
  auto pos1 = std::make_shared<inner::TargetNode>();
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         shoulder1, pos1->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         shoulder2, pos1->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         elbow1, pos1->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         elbow2, pos1->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct, pos1->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct, pos1->tgt_info.tgt_pos);

  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE; }, shoulder_mv, pos1->tgt_info.profile_fn);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW; }, elbow_mv, pos1->tgt_info.profile_fn);

  auto pos2 = std::make_shared<inner::TargetNode>();
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         shoulder2, pos2->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         shoulder1, pos2->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         elbow2, pos2->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         elbow1, pos2->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct, pos2->tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct, pos2->tgt_info.tgt_pos);

  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE; }, shoulder_mv, pos2->tgt_info.profile_fn);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW; }, elbow_mv, pos2->tgt_info.profile_fn);
  // clang-format on

  pos1->next = pos2;
  pos2->next = pos1;

  inner::chain_nodes(pos1, duration_sec, spot);
}

}  // namespace sp::walk