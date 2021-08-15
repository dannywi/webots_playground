#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

using namespace std;
using namespace webots;

namespace sp {

const int TIME_STEP_MS = 64;

template <typename T, int N = 12>
using joint_array = array<T, N>;

const joint_array<string> motor_names = {"front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
                                         "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
                                         "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
                                         "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"};

struct Spot {
  Spot() : me(make_shared<Robot>()) {
    // Turn on cameras
    const int sampling_period = sp::TIME_STEP_MS * 2;
    me->getCamera("left head camera")->enable(sampling_period);
    me->getCamera("right head camera")->enable(sampling_period);

    // Get legs
    transform(sp::motor_names.begin(), sp::motor_names.end(), legs.begin(), [this](const auto& name) { return this->me->getMotor(name); });
  }

  void step() {
    if (me->step(TIME_STEP_MS) == -1) {
      exit(0);
    }
  }

  shared_ptr<Robot> me;
  joint_array<Motor*> legs;
};

void movement_decomposition(const joint_array<double>& tgt_pos, double duration_sec, Spot& spot) {
  const double steps_to_achieve_target = duration_sec * 1000 / TIME_STEP_MS;

  // Get init positions and incremental steps
  joint_array<double> init_pos;
  joint_array<double> incr_pos;
  for (size_t m = 0; m < spot.legs.size(); ++m) {
    init_pos[m] = spot.legs[m]->getTargetPosition();
    incr_pos[m] = (tgt_pos[m] - init_pos[m]) / steps_to_achieve_target;
  }

  // Increment positions until they reach target
  for (size_t s = 0; s < steps_to_achieve_target; ++s) {
    for (size_t m = 0; m < spot.legs.size(); ++m) {
      spot.legs[m]->setPosition(init_pos[m] + ((1 + s) * incr_pos[m]));
    }
    spot.step();
  }
}

static void lie_down(double duration, Spot& spot) {
  const joint_array<double> tgt_pos = {-0.40, -0.99, 1.59,   // Front left leg
                                       0.40,  -0.99, 1.59,   // Front right leg
                                       -0.40, -0.99, 1.59,   // Rear left leg
                                       0.40,  -0.99, 1.59};  // Rear right leg
  movement_decomposition(tgt_pos, duration, spot);
}

void stand_up(double duration, Spot& spot) {
  const joint_array<double> tgt_pos = {-0.1, 0.0, 0.0,   // Front left leg
                                       0.1,  0.0, 0.0,   // Front right leg
                                       -0.1, 0.0, 0.0,   // Rear left leg
                                       0.1,  0.0, 0.0};  // Rear right leg

  movement_decomposition(tgt_pos, duration, spot);
}

void sit_down(double duration, Spot& spot) {
  const joint_array<double> tgt_pos = {-0.20, -0.40, -0.19,  // Front left leg
                                       0.20,  -0.40, -0.19,  // Front right leg
                                       -0.40, -0.90, 1.18,   // Rear left leg
                                       0.40,  -0.90, 1.18};  // Rear right leg

  movement_decomposition(tgt_pos, duration, spot);
}

void give_paw(Spot& spot) {
  // Stabilize posture
  const joint_array<double> tgt_pos1 = {-0.20, -0.30, 0.05,   // Front left leg
                                        0.20,  -0.40, -0.19,  // Front right leg
                                        -0.40, -0.90, 1.18,   // Rear left leg
                                        0.49,  -0.90, 0.80};  // Rear right leg

  movement_decomposition(tgt_pos1, 2, spot);

  const double initial_time = spot.me->getTime();
  while (spot.me->getTime() - initial_time < 8) {
    spot.legs[4]->setPosition(0.2 * sin(2 * spot.me->getTime()) + 0.6);  // Upperarm movement
    spot.legs[5]->setPosition(0.4 * sin(2 * spot.me->getTime()));        // Forearm movement
    spot.step();
  }

  sit_down(3, spot);
}

}  // namespace sp

int main(int argc, char** argv) {
  sp::Spot spot;

  sp::sit_down(2, spot);
  sp::give_paw(spot);
  sp::stand_up(0.05, spot);
}
