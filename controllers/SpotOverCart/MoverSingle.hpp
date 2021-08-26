#pragma once

#include "BasicTypes.hpp"
#include "Utils.hpp"

namespace sp::single {

namespace inner {
void move_decomposed(const joint_array<double>& tgt_pos, double duration_sec, Spot& spot) {
  const double steps_to_achieve_target = duration_sec * 1000 / TIME_STEP_MS;

  // Get init positions and incremental steps
  joint_array<double> init_pos;
  joint_array<double> incr_pos;
  for (size_t m = 0; m < spot.leg_motors.size(); ++m) {
    init_pos[m] = spot.leg_motors[m]->getTargetPosition();
    incr_pos[m] = (tgt_pos[m] - init_pos[m]) / steps_to_achieve_target;
  }

  // Increment positions until they reach target
  for (size_t s = 0; s < steps_to_achieve_target; ++s) {
    for (size_t m = 0; m < spot.leg_motors.size(); ++m) {
      spot.leg_motors[m]->setPosition(init_pos[m] + ((1 + s) * incr_pos[m]));
    }
    spot.step();
  }
}
}  // namespace inner

void lie_down(double duration_sec, Spot& spot) {
  const joint_array<double> tgt_pos = {-0.40, -0.99, 1.59,   // Front left leg
                                       0.40,  -0.99, 1.59,   // Front right leg
                                       -0.40, -0.99, 1.59,   // Rear left leg
                                       0.40,  -0.99, 1.59};  // Rear right leg
  inner::move_decomposed(tgt_pos, duration_sec, spot);
}

void stand_up(double duration_sec, Spot& spot) {
  const joint_array<double> tgt_pos = {-0.1, 0.0, 0.0,   // Front left leg
                                       0.1,  0.0, 0.0,   // Front right leg
                                       -0.1, 0.0, 0.0,   // Rear left leg
                                       0.1,  0.0, 0.0};  // Rear right leg

  inner::move_decomposed(tgt_pos, duration_sec, spot);
}

void sit_down(double duration_sec, Spot& spot) {
  const joint_array<double> tgt_pos = {-0.20, -0.40, -0.19,  // Front left leg
                                       0.20,  -0.40, -0.19,  // Front right leg
                                       -0.40, -0.90, 1.18,   // Rear left leg
                                       0.40,  -0.90, 1.18};  // Rear right leg

  inner::move_decomposed(tgt_pos, duration_sec, spot);
}

void give_paw(Spot& spot) {
  // Stabilize posture
  const joint_array<double> tgt_pos1 = {-0.20, -0.30, 0.05,   // Front left leg
                                        0.20,  -0.40, -0.19,  // Front right leg
                                        -0.40, -0.90, 1.18,   // Rear left leg
                                        0.49,  -0.90, 0.80};  // Rear right leg

  inner::move_decomposed(tgt_pos1, 2, spot);

  const double initial_time = spot.me->getTime();
  while (spot.me->getTime() - initial_time < 6) {
    spot.leg_motors[4]->setPosition(0.2 * sin(2 * spot.me->getTime()) + 0.6);  // Upperarm movement
    spot.leg_motors[5]->setPosition(0.4 * sin(2 * spot.me->getTime()));        // Forearm movement
    spot.step();
  }

  sit_down(3, spot);
}

void jiggle(size_t duration_sec, Spot& spot) {
  // remember initial pos
  joint_array<double> init_pos;
  for (size_t m = 0; m < spot.leg_motors.size(); ++m) init_pos[m] = spot.leg_motors[m]->getTargetPosition();

  auto create_target = [init_pos]() {
    const double deviation = 0.05;
    auto new_pos = init_pos;
    for (size_t i = 0; i < init_pos.size(); ++i) {
      auto range = motor::type_range[motor::types[i]];
      new_pos[i] = init_pos[i] + sp::util::get_random(deviation) * (range.max - range.min);
      new_pos[i] = std::min(range.max, std::max(range.min, new_pos[i]));
    }

    return new_pos;
  };

  const double initial_time = spot.me->getTime();
  double last_start_time = initial_time;
  const double interval_sec = 2.0;
  auto rand_interval = [interval_sec]() { return interval_sec * (1.0 + sp::util::get_random(0.1)); };
  joint_array<double> new_pos = create_target();
  while (spot.me->getTime() - initial_time < duration_sec) {
    if (spot.me->getTime() - last_start_time > interval_sec) {
      // interval passed, get new position
      new_pos = create_target();
      last_start_time = spot.me->getTime();
    }
    inner::move_decomposed(new_pos, rand_interval(), spot);
    spot.step();
  }

  // back to original pos, to stabilize next move
  inner::move_decomposed(init_pos, rand_interval(), spot);
}

void fall_over(double duration_sec, Spot& spot) {
  const joint_array<double> tgt_pos = {0.4, -1.0, 1.0,   // Front left leg
                                       0.1,  0.0, -0.4,   // Front right leg
                                       0.4, -1.0, 1.0,   // Rear left leg
                                       0.1,  0.0, -0.4};  // Rear right leg

  inner::move_decomposed(tgt_pos, duration_sec, spot);
}

}  // namespace sp::single
