# WEBOTS Projects
A collection of webots projects and controllers. Some controllers use C++14 and above, and cannot be compiled from Webots' GUI. E.g. for the collision avoidance controller, do these to compile (tested on MAC):
```
cd controllers/EPuckAvoidCollision

# set installation path (used in Makefile)
export WEBOTS_HOME=/Applications/Webots.app

# note that the Makefile is adjusted to use C++17
make
```