#pragma once

#include <memory>
#include <random>

namespace sp::util {

namespace inner {
template <typename T>
T get_random_sub(T range, std::mt19937& generator);

template <>
double get_random_sub<double>(double range, std::mt19937& generator) {
  std::uniform_real_distribution<double> distribution(-range, range);
  return distribution(generator);
}

template <>
int get_random_sub<int>(int range, std::mt19937& generator) {
  std::uniform_int_distribution<int> distribution(-range, range);
  return distribution(generator);
}
}  // namespace inner

// gives random number [-range, range]
template <typename T>
T get_random(T range) {
  static std::random_device os_seed;
  static const uint_least32_t seed = os_seed();
  static std::mt19937 generator(seed);
  return inner::get_random_sub(range, generator);
}

}  // namespace sp::util