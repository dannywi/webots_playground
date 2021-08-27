#include <iostream>

#include "BasicTypes.hpp"
#include "MoverSingle.hpp"
#include "MoverWalk.hpp"
#include "Utils.hpp"

using namespace std;

namespace sp {
void try_jumping(Spot& spot) {
  sp::single::lie_down(1, spot);
  sp::single::sit_down(2, spot);
  sp::single::give_paw(spot);
  sp::single::stand_up(0.05, spot);
}

void uneasy(Spot& spot) {
  sp::single::stand_up(1, spot);
  sp::single::jiggle(10, spot);
  sp::single::sit_down(1, spot);
  sp::single::jiggle(5, spot);
}

void walk_v1(Spot& spot) {
  sp::walk::spread_legs(2, spot);
  sp::walk::walk_v1(3, spot);
}

void turn_left(Spot& spot) { sp::walk::turn_v1(sp::Side::LEFT, 5, spot); }

void move_sideways_back_and_forth(Spot& spot) {
  while (true) {
    sp::single::stand_up(1, spot);
    sp::walk::move_sideways(sp::Side::RIGHT, 50, spot);
    sp::single::stand_up(1, spot);
    sp::walk::move_sideways(sp::Side::LEFT, 50, spot);
  }
}
};  // namespace sp

int main(int argc, char** argv) {
  sp::Spot spot;

  // sp::single::fall_over(0.3, spot);
  sp::walk::walk_v2(10, spot);
  sp::walk::move_sideways(sp::Side::RIGHT, 10, spot);
  sp::walk::turn_v1(sp::Side::RIGHT, 10, spot);
  sp::single::jiggle(3, spot);
  sp::walk::turn_v1(sp::Side::LEFT, 5, spot);
  sp::walk::walk_v2(100, spot);
}
