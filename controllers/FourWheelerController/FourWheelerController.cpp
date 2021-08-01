// File:          FourWheelerController.cpp
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
#include <random>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

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

  // get the time step of the current world.
  // int time_step = (int)robot->getBasicTimeStep();
  // cout << "DEBUG ... time_step [" << time_step << "]" << endl;
  const int time_step = 64;

  // initialize motors
  array<Motor*, 2> wheels_left, wheels_right;
  auto fill_wheel_fn = [&robot](const array<const char*, 2>& names, auto& wheels) {
    transform(names.begin(), names.end(), wheels.begin(), [&robot](auto name) { return robot->getMotor(name); });
  };
  fill_wheel_fn({"wheel_front_left", "wheel_rear_left"}, wheels_left);
  fill_wheel_fn({"wheel_front_right", "wheel_rear_right"}, wheels_right);

  // initialize sensors
  auto sensor_names = {"dist_sensor_left", "dist_sensor_right"};

  array<DistanceSensor*, 2> sensors;
  transform(sensor_names.begin(), sensor_names.end(), sensors.begin(),
            [&robot](const char* sensor_name) { return robot->getDistanceSensor(sensor_name); });
  for (auto sensor : sensors) sensor->enable(time_step);

  auto init_fn = [](auto wheels) {
    for (auto wheel : wheels) {
      wheel->setPosition(INFINITY);
      wheel->setVelocity(0);
    }
  };
  init_fn(wheels_left);
  init_fn(wheels_right);

  const double SPEED = 2.5;  // rad/sec
  unsigned short avoid_counter = 0;

  default_random_engine generator;
  uniform_int_distribution<int> distribution(80, 160);
  uniform_int_distribution<int> direction_dice(1, 2);
  bool turn_left = true;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(time_step) != -1) {
    double speed_left = SPEED;
    double speed_right = SPEED;

    if (sensors[0]->getValue() < 1000 || sensors[1]->getValue() < 1000)
      cout << "DEBUG ... sensors L[" << sensors[0]->getValue() << "] R[" << sensors[1]->getValue() << "]" << endl;

    if (avoid_counter > 0) {
      --avoid_counter;
    } else {
      for (auto sensor : sensors)
        if (sensor->getValue() < 950) {
          avoid_counter = distribution(generator) / SPEED;
          turn_left = direction_dice(generator) == 1;
          cout << "DEBUG ... avoid counter [" << avoid_counter << "] turn_left [" << turn_left << "]" << endl;
          break;
        }
    }

    if (avoid_counter > 0) {
      if (turn_left)
        speed_left *= -1;
      else
        speed_right *= -1;
    }

    for (auto wheel : wheels_left) wheel->setVelocity(speed_left);
    for (auto wheel : wheels_right) wheel->setVelocity(speed_right);
  };

  return 0;
}
