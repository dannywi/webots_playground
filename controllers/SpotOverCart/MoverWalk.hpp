#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "BasicTypes.hpp"
#include "Profile.hpp"
#include "Utils.hpp"

namespace sp::walk {

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

using joint_mv_pair = std::pair<joint_array<double>, joint_array<std::function<double(double)>>>;
const double UNDEF = std::numeric_limits<double>::max();

// ideas:
//   - add conditional to go to next, some kind of state management
//     - use to stop or skip to next early
struct TargetNode {
  TargetNode() {}
  TargetNode(const TargetInfo& tgt_info, const std::shared_ptr<TargetNode>& next) : tgt_info(tgt_info), next(next) {}

  TargetInfo tgt_info;
  std::shared_ptr<TargetNode> next{nullptr};

  static std::shared_ptr<TargetNode> link(std::vector<joint_mv_pair> raw_list, bool looper = false) {
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

joint_array<std::vector<double>> get_incremental_steps(const joint_array<double>& init_pos, const joint_array<double>& tgt_pos, size_t num_steps,
                                                       const joint_array<std::function<double(double)>>& profile_fn) {
  joint_array<std::vector<double>> result;
  for (size_t i = 0; i < init_pos.size(); ++i) {
    if (tgt_pos[i] == UNDEF) {  // can use optional, if not too verbose
      std::vector<double> no_move(num_steps, init_pos[i]);
      result[i] = no_move;
      continue;
    }
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
    move_decomposed(curr->tgt_info, duration_sec, spot);
    curr = curr->next;
  }
}

}  // namespace inner

void base_position(double duration_sec, Spot& spot) {
  inner::TargetInfo tgt_info;

  double abduct = 0.15;
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct, tgt_info.tgt_pos);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct, tgt_info.tgt_pos);

  std::vector<std::function<double(double)>> funcs{fn::linear, fn::square, fn::cube, fn::tanh4, fn::tanh7};
  auto pick = [funcs]() { return funcs[abs(static_cast<int>(sp::util::get_random(static_cast<int>(funcs.size() - 1))))]; };
  for (auto& fn : tgt_info.profile_fn) fn = pick();

  inner::move_decomposed(tgt_info, duration_sec, spot);
}

void spread_legs(double duration_sec, Spot& spot) {
  inner::TargetInfo tgt_info;
  const double abduct = 0.1;
  tgt_info.tgt_pos = {-abduct, -0.4, 0.3,   // Front left leg
                      abduct,  -0.4, 0.3,   // Front right leg
                      -abduct, -0.4, 0.3,   // Rear left leg
                      abduct,  -0.4, 0.3};  // Rear right leg
  inner::move_decomposed(tgt_info, duration_sec, spot);
}

// chain 2 positions and use move functions to make the states in between form a "walking" curve
void walk_v1(double duration_sec, Spot& spot) {
  // template position, legs spread
  joint_array<double> base;
  base.fill(0);
  const double abduct = 0.2;
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct, base);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct, base);

  const double shoulderB = -0.3;
  const double shoulderF = 0.4;
  const double elbowB = 0.3;
  const double elbowF = -0.2;

  // diagonal opposites
  auto FL_BR = [](auto p, auto s) { return (p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT); };
  auto FR_BL = [](auto p, auto s) { return (p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT); };

  // move diagonal opposite pairs forwards and backwards
  joint_array<double> tgt1 = base;
  motor::set_joints([&](auto p, auto s, auto t) { return FL_BR(p, s) && t == Type::SHOULDER_ROTATE; }, shoulderB, tgt1);
  motor::set_joints([&](auto p, auto s, auto t) { return FL_BR(p, s) && t == Type::ELBOW; }, elbowB, tgt1);
  motor::set_joints([&](auto p, auto s, auto t) { return FR_BL(p, s) && t == Type::SHOULDER_ROTATE; }, shoulderF, tgt1);
  motor::set_joints([&](auto p, auto s, auto t) { return FR_BL(p, s) && t == Type::ELBOW; }, elbowF, tgt1);

  // swap the position (diagonal mirror of the above)
  joint_array<double> tgt2 = base;
  motor::set_joints([&](auto p, auto s, auto t) { return FL_BR(p, s) && t == Type::SHOULDER_ROTATE; }, shoulderF, tgt2);
  motor::set_joints([&](auto p, auto s, auto t) { return FL_BR(p, s) && t == Type::ELBOW; }, elbowF, tgt2);
  motor::set_joints([&](auto p, auto s, auto t) { return FR_BL(p, s) && t == Type::SHOULDER_ROTATE; }, shoulderB, tgt2);
  motor::set_joints([&](auto p, auto s, auto t) { return FR_BL(p, s) && t == Type::ELBOW; }, elbowB, tgt2);

  joint_array<std::function<double(double)>> default_mv;
  default_mv.fill(fn::linear);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE; }, fn::tanh4, default_mv);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::ELBOW; }, fn::square, default_mv);

  auto first = inner::TargetNode::link(
      {
          {tgt1, default_mv},
          {tgt2, default_mv},
      },
      true);

  inner::chain_nodes(first, duration_sec, spot);
}

void turn_left_v1(Spot& spot, double speed_ratio = 1) {
  const double default_sec = 0.3;
  double duration_sec = std::max(0.0, std::min(3.0, default_sec / speed_ratio));

  const double shoulder = -0.2;
  const double elbow = 0.35;
  const double abduct_out = 0.2;
  const double abduct_in = 0.15;

  // clang-format off
  // Standing with slight leg spread
  joint_array<double> tgt1;
  tgt1.fill(0);

  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -abduct_out, tgt1);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abduct_out, tgt1);

  // Lifting diagonally opposite legs slightly up and inwards
  joint_array<double> tgt2 = tgt1;
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   0, tgt2);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   shoulder, tgt2);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   elbow, tgt2);

  // Landing the legs slightly inwards
  joint_array<double> tgt3 = tgt1;
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && ((p == Pos::FRONT && s == Side::RIGHT) || (p == Pos::BACK && s == Side::LEFT)); },
                   abduct_in, tgt3);

  // Lifting the other leg pair
  joint_array<double> tgt4 = tgt1;
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                   shoulder, tgt4);
  motor::set_joints([](auto p, auto s, auto t) { return t == Type::ELBOW && ((p == Pos::FRONT && s == Side::LEFT) || (p == Pos::BACK && s == Side::RIGHT)); },
                   elbow, tgt4);
  // clang-format on

  joint_array<std::function<double(double)>> linear;
  linear.fill(fn::linear);
  auto first = inner::TargetNode::link(
      {
          {tgt1, linear},
          {tgt2, linear},
          {tgt3, linear},
          {tgt4, linear},
      },
      true);

  inner::chain_nodes(first, duration_sec, spot);
}

namespace inner {
// pass multiple rows of legs filter, target position, move function
joint_mv_pair gen_pos(std::vector<std::tuple<std::function<bool(Pos, Side, Type)>, double, std::function<double(double)>>> info) {
  joint_array<double> tgt;
  tgt.fill(UNDEF);  // default no move
  joint_array<std::function<double(double)>> move_fn;
  move_fn.fill(fn::linear);

  for (const auto& i : info) {
    motor::set_joints(std::get<0>(i), std::get<1>(i), tgt);
    motor::set_joints(std::get<0>(i), std::get<2>(i), move_fn);
  }

  return std::make_pair(tgt, move_fn);
}

struct LegPos {
  Pos p;
  Side s;
  double shoulder_abd;
  std::function<double(double)> shoulder_abd_fn;
  double shoulder_rot;
  std::function<double(double)> shoulder_rot_fn;
  double elbow;
  std::function<double(double)> elbow_fn;
};

joint_mv_pair gen_pos_leg(LegPos leg) {
  joint_array<double> tgt;
  tgt.fill(UNDEF);  // default no move
  joint_array<std::function<double(double)>> move_fn;
  move_fn.fill(fn::linear);

  motor::set_joints([&](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && p == leg.p && s == leg.s; }, leg.shoulder_abd, tgt);
  motor::set_joints([&](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && p == leg.p && s == leg.s; }, leg.shoulder_abd_fn, move_fn);
  motor::set_joints([&](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && p == leg.p && s == leg.s; }, leg.shoulder_rot, tgt);
  motor::set_joints([&](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && p == leg.p && s == leg.s; }, leg.shoulder_rot_fn, move_fn);
  motor::set_joints([&](auto p, auto s, auto t) { return t == Type::ELBOW && p == leg.p && s == leg.s; }, leg.elbow, tgt);
  motor::set_joints([&](auto p, auto s, auto t) { return t == Type::ELBOW && p == leg.p && s == leg.s; }, leg.elbow_fn, move_fn);

  return std::make_pair(tgt, move_fn);
}

}  // namespace inner

void move_sideways(Side side, size_t times, Spot& spot, double speed_ratio = 1.0) {
  std::vector<inner::joint_mv_pair> raw_list;
  const double default_sec = 0.3;
  double duration_sec = std::max(0.0, std::min(3.0, default_sec / speed_ratio));

  // template position, legs spread, leaned down a bit to lower center of gravity
  auto pos1 = inner::gen_pos({
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -0.1, fn::linear),
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, 0.1, fn::linear),
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE; }, -0.3, fn::linear),
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::ELBOW; }, 0.4, fn::linear),
  });

  // lift one side, bending knees on both sides
  double abd_left, abd_right;
  std::tie(abd_left, abd_right) = side == Side::RIGHT ? std::make_tuple(0.5, 0.25) : std::make_tuple(-0.25, -0.5);
  auto pos2 = inner::gen_pos({
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, abd_left, fn::linear),
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abd_right, fn::linear),
      std::make_tuple([&](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE; }, -0.4, fn::linear),
      std::make_tuple([&](auto p, auto s, auto t) { return t == Type::ELBOW; }, 0.6, fn::linear),
  });

  for (size_t i = 0; i < times; ++i) {
    raw_list.emplace_back(pos1);
    raw_list.emplace_back(pos2);
  }

  raw_list.emplace_back(pos1);

  inner::chain_nodes(inner::TargetNode::link(raw_list), duration_sec, spot);
}

void walk_v2(size_t step_sets, Spot& spot, double speed_ratio = 1.0) {
  const double default_sec = 0.3;
  double duration_sec = std::max(0.0, std::min(3.0, default_sec / speed_ratio));

  std::vector<inner::joint_mv_pair> raw_list;

  // base position, legs spread, leaned down a bit to lower center of gravity
  raw_list.emplace_back(inner::gen_pos({
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, -0.1, fn::linear),
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, 0.1, fn::linear),
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE; }, -0.3, fn::linear),
      std::make_tuple([](auto p, auto s, auto t) { return t == Type::ELBOW; }, 0.4, fn::linear),
  }));
  inner::chain_nodes(inner::TargetNode::link(raw_list, false), duration_sec, spot);

  // walking loop starts here
  raw_list.clear();

  auto lean = [](Side side, auto& raw_list) {
    double abd_l = -0.1;
    double abd_r = 0.1;
    double rot = -0.3;
    double elb = 0.4;
    if(side == Side::LEFT) {
      abd_l += 0.2;
    } else {
      abd_r -= 0.2;
    }
    raw_list.emplace_back(inner::gen_pos({
        std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::LEFT; }, abd_l, fn::linear),
        std::make_tuple([](auto p, auto s, auto t) { return t == Type::SHOULDER_ABDUCT && s == Side::RIGHT; }, abd_r, fn::linear),
        std::make_tuple([&](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && s != side; }, rot - 0.1, fn::linear),
        std::make_tuple([&](auto p, auto s, auto t) { return t == Type::ELBOW && s != side; }, elb - 0.1, fn::linear),
        std::make_tuple([&](auto p, auto s, auto t) { return t == Type::SHOULDER_ROTATE && s == side; }, rot, fn::linear),
        std::make_tuple([&](auto p, auto s, auto t) { return t == Type::ELBOW && s == side; }, elb, fn::linear),
    }));
  };

  auto leg_fwd = [](Pos pos, Side side, auto& raw_list) {
    raw_list.emplace_back(inner::gen_pos_leg({pos, side, inner::UNDEF, fn::linear, 0.1, fn::tanh7, 0.6, fn::square_inv}));
    raw_list.emplace_back(inner::gen_pos_leg({pos, side, inner::UNDEF, fn::linear, 0.3, fn::tanh7, 0.1, fn::square}));
  };

  for(size_t i = 0; i < step_sets; ++i) {
  lean(Side::LEFT, raw_list);
  leg_fwd(Pos::FRONT, Side::RIGHT, raw_list);
  lean(Side::RIGHT, raw_list);
  leg_fwd(Pos::BACK, Side::LEFT, raw_list);
  leg_fwd(Pos::FRONT, Side::LEFT, raw_list);
  lean(Side::LEFT, raw_list);
  leg_fwd(Pos::BACK, Side::RIGHT, raw_list);
  }
  inner::chain_nodes(inner::TargetNode::link(raw_list, false), duration_sec, spot);
}

}  // namespace sp::walk