#pragma once

#include <cmath>

namespace InputUtils {
typedef struct {
  double x;
  double y;
  bool deadzoneApplied;
} DeadzoneAxes;

/**
 * @brief Remaps x/y inputs to a combined value before applying a deadzone
 * 
 * @param x 
 * @param y 
 * @param deadzoneDistance 
 * @return DeadzoneAxes 
 */
DeadzoneAxes CalculateCircularDeadzone(double x, double y, double deadzoneDistance) {
  if (std::hypot(x, y) > deadzoneDistance) {
    return {.x = x, .y = y, .deadzoneApplied = false};
  }

  return {.x = 0, .y = 0, .deadzoneApplied = true};
}
}  // namespace InputUtils