#include "kinematic_models.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

double wrapAngleLocal(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

} // namespace

AckermannKinematicModel::AckermannKinematicModel(
    const AckermannKinematicsConfig &config)
    : config_(config) {
  if (config_.wheelbase <= 0.0) {
    throw std::invalid_argument("Ackermann wheelbase must be > 0");
  }
  if (config_.linear_step <= 0.0) {
    throw std::invalid_argument("Ackermann linear_step must be > 0");
  }
}

std::vector<MotionPrimitive>
AckermannKinematicModel::buildMotionPrimitives(int branching_factor) const {
  std::vector<MotionPrimitive> primitives;
  const int branches = std::max(1, branching_factor);
  primitives.reserve(static_cast<std::size_t>(branches));

  if (branches == 1) {
    primitives.push_back(
        MotionPrimitive{config_.linear_step, 0.0, 0.0, false});
    return primitives;
  }

  const double angle_step =
      (2.0 * config_.max_steering_angle) / static_cast<double>(branches - 1);
  for (int i = 0; i < branches; ++i) {
    const double steering =
        -config_.max_steering_angle + static_cast<double>(i) * angle_step;
    primitives.push_back(
        MotionPrimitive{config_.linear_step, 0.0, steering, false});
  }
  return primitives;
}

RolloutResult AckermannKinematicModel::rollout(const State &start,
                                               const MotionPrimitive &primitive,
                                               int steps) const {
  RolloutResult result;
  result.samples.reserve(static_cast<std::size_t>(std::max(0, steps)));

  State state = start;
  for (int i = 0; i < steps; ++i) {
    const double dtheta =
        std::tan(primitive.steering_angle) / config_.wheelbase;
    state.x += primitive.linear_step * std::cos(state.heading);
    state.y += primitive.linear_step * std::sin(state.heading);
    state.heading =
        wrapAngleLocal(state.heading + primitive.linear_step * dtheta);
    result.samples.push_back(state);
  }

  return result;
}

double AckermannKinematicModel::controlEffort(
    const MotionPrimitive &primitive) const {
  return std::fabs(primitive.steering_angle);
}

double AckermannKinematicModel::smoothnessCost(
    const MotionPrimitive *previous, const MotionPrimitive &current) const {
  if (previous == nullptr) {
    return std::fabs(current.steering_angle);
  }
  return std::fabs(current.steering_angle - previous->steering_angle);
}

double AckermannKinematicModel::maxForwardStep() const {
  return config_.linear_step;
}

DifferentialKinematicModel::DifferentialKinematicModel(
    const DifferentialKinematicsConfig &config)
    : config_(config) {
  if (config_.linear_step <= 0.0) {
    throw std::invalid_argument("Differential linear_step must be > 0");
  }
  if (config_.max_angular_step < 0.0) {
    throw std::invalid_argument("Differential max_angular_step must be >= 0");
  }
}

std::vector<MotionPrimitive>
DifferentialKinematicModel::buildMotionPrimitives(int branching_factor) const {
  const int moving_samples =
      std::max(1, config_.moving_angular_samples > 0
                      ? config_.moving_angular_samples
                      : branching_factor);

  std::vector<MotionPrimitive> primitives;
  primitives.reserve(static_cast<std::size_t>(
      moving_samples + (config_.include_in_place_rotation ? 2 : 0)));

  if (moving_samples == 1 || config_.max_angular_step == 0.0) {
    primitives.push_back(MotionPrimitive{config_.linear_step, 0.0, 0.0, false});
  } else {
    const double angular_step =
        (2.0 * config_.max_angular_step) / static_cast<double>(moving_samples - 1);
    for (int i = 0; i < moving_samples; ++i) {
      const double angular =
          -config_.max_angular_step + static_cast<double>(i) * angular_step;
      primitives.push_back(
          MotionPrimitive{config_.linear_step, angular, 0.0, false});
    }
  }

  if (config_.include_in_place_rotation && config_.max_angular_step > 0.0) {
    primitives.push_back(
        MotionPrimitive{0.0, config_.max_angular_step, 0.0, true});
    primitives.push_back(
        MotionPrimitive{0.0, -config_.max_angular_step, 0.0, true});
  }

  return primitives;
}

RolloutResult DifferentialKinematicModel::rollout(
    const State &start, const MotionPrimitive &primitive, int steps) const {
  RolloutResult result;
  result.samples.reserve(static_cast<std::size_t>(std::max(0, steps)));

  State state = start;
  for (int i = 0; i < steps; ++i) {
    if (primitive.in_place_rotation) {
      state.heading = wrapAngleLocal(state.heading + primitive.angular_step);
    } else {
      const double mid_heading = state.heading + 0.5 * primitive.angular_step;
      state.x += primitive.linear_step * std::cos(mid_heading);
      state.y += primitive.linear_step * std::sin(mid_heading);
      state.heading = wrapAngleLocal(state.heading + primitive.angular_step);
    }
    result.samples.push_back(state);
  }

  return result;
}

double DifferentialKinematicModel::controlEffort(
    const MotionPrimitive &primitive) const {
  return std::fabs(primitive.angular_step) +
         (primitive.in_place_rotation ? std::fabs(primitive.angular_step) : 0.0);
}

double DifferentialKinematicModel::smoothnessCost(
    const MotionPrimitive *previous, const MotionPrimitive &current) const {
  if (previous == nullptr) {
    return std::fabs(current.angular_step);
  }

  double penalty = std::fabs(current.angular_step - previous->angular_step);
  if (previous->in_place_rotation != current.in_place_rotation) {
    penalty += std::fabs(current.angular_step);
  }
  return penalty;
}

double DifferentialKinematicModel::maxForwardStep() const {
  return config_.linear_step;
}

std::unique_ptr<KinematicModel>
makeKinematicModel(const std::string &model_name,
                   const AckermannKinematicsConfig &ackermann_config,
                   const DifferentialKinematicsConfig &differential_config) {
  if (model_name == "ackermann") {
    return std::make_unique<AckermannKinematicModel>(ackermann_config);
  }
  if (model_name == "differential") {
    return std::make_unique<DifferentialKinematicModel>(differential_config);
  }

  throw std::invalid_argument("Unsupported kinematics.model: " + model_name);
}
