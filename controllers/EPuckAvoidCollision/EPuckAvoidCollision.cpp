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

bool is_obstacle(const array<DistanceSensor*, 8>& ps,
                 const array<size_t, 3>& indices) {
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
    snprintf(buf, MAX_BUF, "%c%s [%.4f %.4f %.4f]", side, (mark_fn(ps) ? "*" : " "),
      ps[idx[0]]->getValue(),
      ps[idx[1]]->getValue(),
      ps[idx[2]]->getValue()
    );
    return string(buf);
  };

  cout << "DEBUG ... " <<
    debug_str_fn('L', left_obstacle, left_sensor_idx) << " " <<
    debug_str_fn('R', right_obstacle, right_sensor_idx) << endl;
}

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
  // Robot *robot = new Robot();
  shared_ptr<Robot> robot = make_shared<Robot>();

  // get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  const array<string, 8> psNames = {"ps0", "ps1", "ps2", "ps3",
                                    "ps4", "ps5", "ps6", "ps7"};

  array<DistanceSensor*, 8> ps;
  // read sensors outputs

  for (size_t i = 0; i < psNames.size(); ++i) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }

  cout << "DEBUGGING .... " << endl;
  auto init_motor = [&robot](auto motor_name) {
    Motor* motor = robot->getMotor(motor_name);
    motor->setPosition(INFINITY);
    motor->setVelocity(0.0);
    return motor;
  };
  Motor* leftMotor = init_motor("left wheel motor");
  Motor* rightMotor = init_motor("right wheel motor");

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    print_front_sensors(ps);

    // initialize motor speeds at 50% of MAX_SPEED.
    double leftSpeed = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;

    // read sensor inputs & modify speeds according to obstacles
    if (left_obstacle(ps)) {
      // turn right
      leftSpeed = 0.5 * MAX_SPEED;
      rightSpeed = -0.5 * MAX_SPEED;
    } else if (right_obstacle(ps)) {
      // turn left
      leftSpeed = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
    }

    // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  };

  // Enter here exit cleanup code.

  // delete robot;
  return 0;
}
