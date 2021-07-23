// File:          EPuckAvoidCollision.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

const unsigned int TIME_STEP = 94;
const double MAX_SPEED = 6.28;
const double SENSOR_THRESHOLD = 78.0;

const array<size_t, 3> left_sensor_idx = {5, 6, 7};
const array<size_t, 3> right_sensor_idx = {0, 1, 2};

namespace webots {

template <size_t SIDE_SENSOR_NUM>
bool is_obstacle(const array<DistanceSensor*, 8>& ps,
                 const array<size_t, SIDE_SENSOR_NUM>& indices) {
  for (auto i : indices) {
    if (ps[i]->getValue() > SENSOR_THRESHOLD) return true;
  }

  return false;
}

bool right_obstacle(const array<DistanceSensor*, 8>& ps) {
  return is_obstacle(ps, right_sensor_idx);
}

bool left_obstacle(const array<DistanceSensor*, 8>& ps) {
  return is_obstacle(ps, left_sensor_idx);
}

// DEBUGGING
void print_front_sensors(const array<DistanceSensor*, 8>& ps) {
  auto debug_str_fn = [&ps](char side, auto mark_fn, auto idx) {
    const size_t MAX_BUF = 100;
    char buf[MAX_BUF];
    snprintf(buf, MAX_BUF, "%c%s [%.4f %.4f %.4f]", side,
             (mark_fn(ps) ? "*" : " "), ps[idx[0]]->getValue(),
             ps[idx[1]]->getValue(), ps[idx[2]]->getValue());
    return string(buf);
  };

  cout << "DEBUG ... " << debug_str_fn('L', left_obstacle, left_sensor_idx)
       << " " << debug_str_fn('R', right_obstacle, right_sensor_idx) << endl;
}

class steering {
 public:
  steering(function<bool(const array<DistanceSensor*, 8>&)> collision_detect_fn,
           bool is_left)
      : collision_detect_fn_(collision_detect_fn), is_left_(is_left) {}

  bool has_obstacle(const array<DistanceSensor*, 8>& ps) const {
    cout << "DEBUG ... has_obs " << (is_left_ ? "left" : "right") << endl;
    bool ret = collision_detect_fn_(ps);
    if (ret)
      cout << "DEBUG ... " << (is_left_ ? "left" : "right") << " has obstacle"
           << endl;
    return ret;
  }

  void adjust_speed(double& speed_left, double& speed_right) const {
    if (is_left_) {
      speed_right *= -1;
    } else {
      speed_left *= -1;
    }
    cout << "DEBUG ... " << (is_left_ ? "left" : "right") << " adjusted ["
         << speed_left << ", " << speed_right << "]" << endl;
  }

 private:
  function<bool(const array<DistanceSensor*, 8>&)> collision_detect_fn_;
  bool is_left_;
};

}  // namespace webots

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char** argv) {
  // create the Robot instance.
  shared_ptr<Robot> robot = make_shared<Robot>();

  const array<string, 8> psNames = {"ps0", "ps1", "ps2", "ps3",
                                    "ps4", "ps5", "ps6", "ps7"};

  // read sensors outputs
  array<DistanceSensor*, 8> ps;
  for (size_t i = 0; i < psNames.size(); ++i) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }

  // initialize steering objects
  steering steer_left{left_obstacle, true};
  steering steer_right{right_obstacle, false};
  reference_wrapper<steering> steer_cur = steer_left;
  reference_wrapper<steering> steer_alt = steer_right;

  // get motors
  auto init_motor = [&robot](auto motor_name) {
    Motor* motor = robot->getMotor(motor_name);
    motor->setPosition(INFINITY);
    motor->setVelocity(0.0);
    return motor;
  };
  Motor* motor_left = init_motor("left wheel motor");
  Motor* motor_right = init_motor("right wheel motor");

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  // TODO:
  // - avoid getting 'stuck'. ideas:
  //   - monitor for several steps, if it's going back and forth, then pick just
  //   one direction
  //   - have STATE, so when in TURNING state, it has to have a stable sensory
  //   readings before transitioning to GO_FORWARD again
  // - LOGS
  // DEBUG ... L  [62.5690 68.9819 69.6186] R* [63.6147 95.8297 65.3230]
  // DEBUG ... has_obs left
  // DEBUG ... has_obs right
  // DEBUG ... right has obstacle
  // DEBUG ... right adjusted [-3.14, 3.14]
  // DEBUG ... L* [94.9351 66.3765 65.3700] R* [73.4618 116.1548 73.1862]
  // DEBUG ... has_obs right
  // DEBUG ... right has obstacle
  // DEBUG ... right adjusted [-3.14, 3.14]
  // DEBUG ... L* [100.3439 72.1449 71.7683] R* [65.6180 85.2676 71.2689]
  // DEBUG ... has_obs right
  // DEBUG ... right has obstacle
  // DEBUG ... right adjusted [-3.14, 3.14]
  // DEBUG ... L* [89.1597 67.8922 63.5797] R  [68.0334 61.2560 58.9287]
  // DEBUG ... has_obs right
  // DEBUG ... has_obs left
  // DEBUG ... left has obstacle
  // DEBUG ... left adjusted [3.14, -3.14]
  
  double speed = 0.5 * MAX_SPEED;
  while (robot->step(TIME_STEP) != -1) {
    print_front_sensors(ps);

    // initialize motor speeds at 50% of MAX_SPEED.
    double speed_left = speed;
    double speed_right = speed;

    if (steer_cur.get().has_obstacle(ps)) {
      steer_cur.get().adjust_speed(speed_left, speed_right);
    } else if (steer_alt.get().has_obstacle(ps)) {
      steer_alt.get().adjust_speed(speed_left, speed_right);
      swap(steer_cur, steer_alt);
    }

    // write actuators inputs
    motor_left->setVelocity(speed_left);
    motor_right->setVelocity(speed_right);
  };

  return 0;
}
