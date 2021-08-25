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
  TargetInfo(const joint_array<double>& tgt_pos, const sp::joint_array<std::function<double(double)>>& profile_fn)
      : tgt_pos(tgt_pos), profile_fn(profile_fn) {}
  joint_array<double> tgt_pos{0};
  joint_array<std::function<double(double)>> profile_fn;
};

// ideas:
//   - add conditional to go to next, some kind of state management
//     - use to stop or skip to next early
struct TargetNode {
  TargetNode() {}
  TargetNode(const TargetInfo& tgt_info, const std::shared_ptr<TargetNode>& next) : tgt_info(tgt_info), next(next) {}

  TargetInfo tgt_info;
  std::shared_ptr<TargetNode> next{nullptr};

  static std::shared_ptr<TargetNode> link(std::vector<std::pair<joint_array<double>, joint_array<std::function<double(double)>>>> raw_list,
                                          bool looper = false) {
    std::shared_ptr<TargetNode> last;
    std::shared_ptr<TargetNode> first;
    for (auto item : raw_list) {
      TargetInfo info{item.first, item.second};
      auto curr = std::make_shared<TargetNode>(info, nullptr);
      if (!first) first = curr;
      if (last) last->next = curr;
      last = curr;
    }

    if (looper) last->next = first;
    return first;
  }
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

void base_position(double duration_sec, Spot& spot) {
  inner::TargetInfo tgt_info;

  double abduct = 0.15;
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct, tgt_info.tgt_pos);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct, tgt_info.tgt_pos);

  std::vector<std::function<double(double)>> funcs{fn::linear, fn::square, fn::cube, fn::tanh4, fn::tanh7};
  auto pick = [funcs]() { return funcs[abs(static_cast<int>(sp::util::get_random(static_cast<int>(funcs.size() - 1))))]; };
  for (auto& fn : tgt_info.profile_fn) fn = pick();

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

// chain 2 positions and use move functions to make the states in between form a "walking" curve
void walk_v1(double duration_sec, Spot& spot) {
  // clang-format off
  // template position, legs spread
  joint_array<double> base;
  base.fill(0);
  const double abduct = 0.2;
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct, base);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct, base);

  const double shoulder1 = -0.3;
  const double shoulder2 = 0.4;
  const double elbow1 = 0.3;
  const double elbow2 = -0.2;

  // move diagonal opposite pairs forwards and backwards
  joint_array<double> tgt1 = base;
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         shoulder1, tgt1);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         shoulder2, tgt1);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         elbow1, tgt1);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         elbow2, tgt1);

  // swap the position (diagonal mirror of the above)
  joint_array<double> tgt2 = base;
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         shoulder2, tgt2);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         shoulder1, tgt2);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                         elbow2, tgt2);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                         elbow1, tgt2);
  // clang-format on

  joint_array<std::function<double(double)>> default_mv;
  default_mv.fill(fn::linear);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE; }, fn::tanh4, default_mv);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW; }, fn::square, default_mv);

  auto first = inner::TargetNode::link(
      {
          {tgt1, default_mv},
          {tgt2, default_mv},
      },
      true);

  inner::chain_nodes(first, duration_sec, spot);
}

void turn_left_v1(double duration_sec, Spot& spot) {
  const double shoulder = -0.2;
  const double elbow = 0.35;
  const double abduct_out = 0.2;
  const double abduct_in = 0.15;

  // clang-format off
  // Standing with slight leg spread
  joint_array<double> tgt1;
  tgt1.fill(0);

  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct_out, tgt1);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct_out, tgt1);

  // Lifting diagonally opposite legs slightly up and inwards
  joint_array<double> tgt2 = tgt1;
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   0, tgt2);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   shoulder, tgt2);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   elbow, tgt2);

  // Landing the legs slightly inwards
  joint_array<double> tgt3 = tgt1;
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   abduct_in, tgt3);

  // Lifting the other leg pair
  joint_array<double> tgt4 = tgt1;
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                   shoulder, tgt4);
  motor::set_joint([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                   elbow, tgt4);

  joint_array<std::function<double(double)>> linear;
  linear.fill(fn::linear);
  auto first = inner::TargetNode::link(
    {
      { tgt1, linear },
      { tgt2, linear },
      { tgt3, linear },
      { tgt4, linear },
    },
    true
  );

  inner::chain_nodes(first, duration_sec, spot);
}

}  // namespace sp::walk