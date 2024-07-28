#include "subzero/drivetrain/SwerveUtils.h"

#include <cmath>
#include <numbers>

using namespace subzero;

double SwerveUtils::StepTowards(double current, double target,
                                double stepsize) {
  if (abs(current - target) <= stepsize) {
    return target;
  } else if (target < current) {
    return current - stepsize;
  }
  return current + stepsize;
}

double SwerveUtils::StepTowardsCircular(double current, double target,
                                        double stepsize) {
  current = WrapAngle(current);
  target = WrapAngle(target);

  double temp = target - current;
  double stepDirection = temp > 0 ? 1 : temp < 0 ? -1 : 0;
  double difference = abs(current - target);

  if (difference <= stepsize) {
    return target;
  } else if (difference > std::numbers::pi) {
    // Handles the case where the difference between the two angles is shorter
    // is less than stepsize when being measured in the other direction
    if (current + 2 * std::numbers::pi - target < stepsize ||
        target + 2 * std::numbers::pi - current < stepsize) {
      return target;
    }
    return WrapAngle(current - stepDirection * stepsize);
  }
  // If the difference is less than pi yet still greater than the step size,
  // we just add stepsize to current
  return current + stepDirection * stepsize;
}

double SwerveUtils::AngleDifference(double angleA, double angleB) {
  double difference = abs(angleA - angleB);
  return difference > std::numbers::pi ? (2 * std::numbers::pi) - difference
                                       : difference;
}

double SwerveUtils::WrapAngle(double angle) {
  double twoPi = 2 * std::numbers::pi;

  if (angle == twoPi) {
    // This case must be handled seperately to avoid floating point errors with
    // the floor after division
    return 0.0;
  } else if (angle > twoPi) {
    double rotations = floor(angle / twoPi);
    // Gets the rotations back into radians and then subtracts angle by the
    // amount of full rotations
    return angle - twoPi * rotations;
  } else if (angle < 0.0) {
    // Must negate the angle which turns it into a positive value. One more
    // rotation is added just in case the result of the floor is zero. Since the
    // _angle is negative, adding it has the effect of subtracting it from those
    // rotations, giving us a valid positive rotation
    double rotations = floor((-angle) / twoPi) + 1;
    return angle + twoPi * rotations;
  }
  return angle;
}