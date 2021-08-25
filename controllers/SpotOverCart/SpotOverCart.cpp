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
};  // namespace sp

int main(int argc, char** argv) {
  sp::Spot spot;

  sp::walk::spread_legs(2, spot);
  sp::walk::walk_version1(3, spot);
  // sp::single::jiggle(50, spot);
}
