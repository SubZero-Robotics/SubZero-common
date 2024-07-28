#include "subzero/drivetrain/SwerveUtils.h"

#include <cmath>
#include <numbers>

using namespace subzero;

double SwerveUtils::StepTowards(double _current, double _target,
                                double _stepsize) {
  if (abs(_current - _target) <= _stepsize) {
    return _target;
  } else if (_target < _current) {
    return _current - _stepsize;
  } else {
    return _current + _stepsize;
  }
}

double SwerveUtils::StepTowardsCircular(double _current, double _target,
                                        double _stepsize) {
  _current = WrapAngle(_current);
  _target = WrapAngle(_target);

  double temp = _target - _current;
  double stepDirection = temp > 0 ? 1 : temp < 0 ? -1 : 0;
  double difference = abs(_current - _target);

  if (difference <= _stepsize) {
    return _target;
  } else if (difference > std::numbers::pi) {
    // Handles the case where the difference between the two angles is shorter
    // is less than _stepsize when being measured in the other direction
    if (_current + 2 * std::numbers::pi - _target < _stepsize ||
        _target + 2 * std::numbers::pi - _current < _stepsize) {
      return _target;
    } else {
      return WrapAngle(_current - stepDirection * _stepsize);
    }
  } else {
    // If the difference is less than pi yet still greater than the step size,
    // we just add stepsize to current
    return _current + stepDirection * _stepsize;
  }
}

double SwerveUtils::AngleDifference(double _angleA, double _angleB) {
  double difference = abs(_angleA - _angleB);
  return difference > std::numbers::pi ? (2 * std::numbers::pi) - difference
                                       : difference;
}

double SwerveUtils::WrapAngle(double _angle) {
  double twoPi = 2 * std::numbers::pi;

  if (_angle == twoPi) {
    // This case must be handled seperately to avoid floating point errors with
    // the floor after division
    return 0.0;
  } else if (_angle > twoPi) {
    double rotations = floor(_angle / twoPi);
    // Gets the rotations back into radians and then subtracts angle by the
    // amount of full rotations
    return _angle - twoPi * rotations;
  } else if (_angle < 0.0) {
    // Must negate the angle which turns it into a positive value. One more
    // rotation is added just in case the result of the floor is zero. Since the
    // _angle is negative, adding it has the effect of subtracting it from those
    // rotations, giving us a valid positive rotation
    double rotations = floor((-_angle) / twoPi) + 1;
    return _angle + twoPi * rotations;
  } else {
    return _angle;
  }
}