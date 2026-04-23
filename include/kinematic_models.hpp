#ifndef KINEMATIC_MODELS_HPP
#define KINEMATIC_MODELS_HPP

#include <memory>
#include <string>
#include <vector>

#include "State.h"

enum class KinematicModelType { Ackermann, Differential };

struct MotionPrimitive {
  double linear_step{0.0};
  double angular_step{0.0};
  double steering_angle{0.0};
  bool in_place_rotation{false};
};

struct RolloutResult {
  std::vector<State> samples;
};

struct AckermannKinematicsConfig {
  double wheelbase{0.0};
  double max_steering_angle{0.0};
  double linear_step{0.0};
};

struct DifferentialKinematicsConfig {
  double linear_step{0.0};
  double max_angular_step{0.0};
  int moving_angular_samples{0};
  bool include_in_place_rotation{true};
};

class KinematicModel {
public:
  virtual ~KinematicModel() = default;

  [[nodiscard]] virtual KinematicModelType type() const = 0;
  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual std::vector<MotionPrimitive>
  buildMotionPrimitives(int branching_factor) const = 0;
  [[nodiscard]] virtual RolloutResult rollout(const State &start,
                                              const MotionPrimitive &primitive,
                                              int steps) const = 0;
  [[nodiscard]] virtual double
  controlEffort(const MotionPrimitive &primitive) const = 0;
  [[nodiscard]] virtual double
  smoothnessCost(const MotionPrimitive *previous,
                 const MotionPrimitive &current) const = 0;
  [[nodiscard]] virtual double maxForwardStep() const = 0;
};

class AckermannKinematicModel final : public KinematicModel {
public:
  explicit AckermannKinematicModel(const AckermannKinematicsConfig &config);

  [[nodiscard]] KinematicModelType type() const override;
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::vector<MotionPrimitive>
  buildMotionPrimitives(int branching_factor) const override;
  [[nodiscard]] RolloutResult rollout(const State &start,
                                      const MotionPrimitive &primitive,
                                      int steps) const override;
  [[nodiscard]] double
  controlEffort(const MotionPrimitive &primitive) const override;
  [[nodiscard]] double
  smoothnessCost(const MotionPrimitive *previous,
                 const MotionPrimitive &current) const override;
  [[nodiscard]] double maxForwardStep() const override;

private:
  AckermannKinematicsConfig config_;
};

class DifferentialKinematicModel final : public KinematicModel {
public:
  explicit DifferentialKinematicModel(
      const DifferentialKinematicsConfig &config);

  [[nodiscard]] KinematicModelType type() const override;
  [[nodiscard]] std::string name() const override;
  [[nodiscard]] std::vector<MotionPrimitive>
  buildMotionPrimitives(int branching_factor) const override;
  [[nodiscard]] RolloutResult rollout(const State &start,
                                      const MotionPrimitive &primitive,
                                      int steps) const override;
  [[nodiscard]] double
  controlEffort(const MotionPrimitive &primitive) const override;
  [[nodiscard]] double
  smoothnessCost(const MotionPrimitive *previous,
                 const MotionPrimitive &current) const override;
  [[nodiscard]] double maxForwardStep() const override;

private:
  DifferentialKinematicsConfig config_;
};

std::unique_ptr<KinematicModel>
makeKinematicModel(const std::string &model_name,
                   const AckermannKinematicsConfig &ackermann_config,
                   const DifferentialKinematicsConfig &differential_config);

#endif // KINEMATIC_MODELS_HPP
