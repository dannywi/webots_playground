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
int main(int argc, char **argv) {
  // create the Robot instance.
  shared_ptr<Robot> robot = make_shared<Robot>();

  // get the time step of the current world.
  // int time_step = (int)robot->getBasicTimeStep();
  // cout << "DEBUG ... time_step [" << time_step << "]" << endl;
  const int time_step = 64;

  // initialize motors
  auto wheel_names = {"wheel_front_left", "wheel_front_right",
                      "wheel_rear_left", "wheel_rear_right"};

  array<Motor *, 4> wheels;
  transform(
      wheel_names.begin(), wheel_names.end(), wheels.begin(),
      [&robot](const char *wheel_name) { return robot->getMotor(wheel_name); });

  double speed = 1.5;  // rad/sec
  for (auto wheel : wheels) {
    wheel->setPosition(INFINITY);
    wheel->setVelocity(speed);
  }

  // initialize sensors
  auto sensor_names = {"dist_sensor_left", "dist_sensor_right"};

  array<DistanceSensor *, 2> sensors;
  transform(sensor_names.begin(), sensor_names.end(), sensors.begin(),
            [&robot](const char *sensor_name) {
              return robot->getDistanceSensor(sensor_name);
            });
  for (auto sensor : sensors) sensor->enable(time_step);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(time_step) != -1) {
    // Read the sensors:
    cout << "DEBUG ... sensors L[" << sensors[0]->getValue() << "] R["
         << sensors[1]->getValue() << "]" << endl;
  };

  return 0;
}
