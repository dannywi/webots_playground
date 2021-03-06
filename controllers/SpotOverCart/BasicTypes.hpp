#pragma once

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

namespace sp {

const int TIME_STEP_MS = 64;

template <typename T, int N = 12>
using joint_array = std::array<T, N>;

enum class Side { RIGHT, LEFT };
enum class Pos { FRONT, BACK };
enum class Type { SHOULDER_ABDUCT, SHOULDER_ROTATE, ELBOW };

namespace motor {
const joint_array<std::string> names = {"front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
                                        "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
                                        "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
                                        "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"};

// clang-format off
const joint_array<Side> sides = {
    Side::LEFT,  Side::LEFT,  Side::LEFT,
    Side::RIGHT, Side::RIGHT, Side::RIGHT,
    Side::LEFT,  Side::LEFT,  Side::LEFT,
    Side::RIGHT, Side::RIGHT, Side::RIGHT,
};

const joint_array<Pos> positions = {
    Pos::FRONT, Pos::FRONT, Pos::FRONT,
    Pos::FRONT, Pos::FRONT, Pos::FRONT,
    Pos::BACK,  Pos::BACK,  Pos::BACK,
    Pos::BACK,  Pos::BACK,  Pos::BACK,
};

const joint_array<Type> types = {
    Type::SHOULDER_ABDUCT, Type::SHOULDER_ROTATE, Type::ELBOW,
    Type::SHOULDER_ABDUCT, Type::SHOULDER_ROTATE, Type::ELBOW,
    Type::SHOULDER_ABDUCT, Type::SHOULDER_ROTATE, Type::ELBOW,
    Type::SHOULDER_ABDUCT, Type::SHOULDER_ROTATE, Type::ELBOW,
};

template <typename T>struct Range { T min; T max; };

std::unordered_map<Type, Range<double>> type_range = {
    {Type::SHOULDER_ABDUCT, {-0.6, 0.5}},
    {Type::SHOULDER_ROTATE, {-1.7, 1.7}},
    {Type::ELBOW, {-0.45, 1.6}},
};
// clang-format on

template <typename ARR_T, typename VAL_T>
void set_joint_value(Pos pos, Side side, Type type, VAL_T value, ARR_T& joints) {
  static std::map<std::tuple<Pos, Side, Type>, size_t> index;
  if (index.empty()) {
    for (size_t i = 0; i < joints.size(); ++i) index[std::make_tuple(positions[i], sides[i], types[i])] = i;
  }

  joints[index[std::make_tuple(pos, side, type)]] = value;
};

template <typename ARR_T, typename VAL_T, typename COND_FN>
void set_joints(COND_FN condition_fn, VAL_T value, ARR_T& joints) {
  for(size_t i = 0; i < joints.size(); ++i) {
    if(condition_fn(positions[i], sides[i], types[i])) {
        joints[i] = value;
    }
  }
};

}  // namespace motor

struct Spot {
  Spot() : me(std::make_shared<webots::Robot>()) {
    // Turn on cameras
    const int sampling_period = sp::TIME_STEP_MS * 2;
    me->getCamera("left head camera")->enable(sampling_period);
    me->getCamera("right head camera")->enable(sampling_period);

    // Get legs
    std::transform(sp::motor::names.begin(), sp::motor::names.end(), leg_motors.begin(),
                   [this](const auto& name) { return this->me->getMotor(name); });
  }

  void step() {
    if (me->step(TIME_STEP_MS) == -1) {
      exit(0);
    }
  }

  std::shared_ptr<webots::Robot> me;
  joint_array<webots::Motor*> leg_motors;
};

}  // namespace sp

template <typename T>
std::ostream& operator<<(std::ostream& os, const sp::joint_array<T>& ar) {
  os << "[";
  for (size_t i = 0; i < ar.size(); ++i) os << ar[i] << (i < ar.size() - 1 ? " " : "");
  os << "]";
  return os;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& ar) {
  os << "[";
  for (size_t i = 0; i < ar.size(); ++i) os << ar[i] << (i < ar.size() - 1 ? " " : "");
  os << "]";
  return os;
}