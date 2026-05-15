#ifndef VEHICLE_FOOTPRINT_HPP
#define VEHICLE_FOOTPRINT_HPP

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

#include "state.hpp"

class VehicleFootprint {
public:
  VehicleFootprint() = default;

  void setDimensions(double axle_to_front, double axle_to_back, double width);
  void createGeometry();

  [[nodiscard]] std::vector<Circle> getCircles(const State &state) const;
  [[nodiscard]] Circle getBoundingCircle(const State &state) const;
  [[nodiscard]] State localToGlobal(const State &reference,
                                    const State &target) const;
  [[nodiscard]] visualization_msgs::msg::MarkerArray
  toMarkerArray(const std::string &frame_id, const rclcpp::Time &stamp) const;

private:
  void setCircles();

  double axle_to_front_{0.0};
  double axle_to_back_{0.0};
  double width_{0.0};

  std::vector<Circle> circles_;
  Circle bounding_circle_;
  geometry_msgs::msg::Polygon vehicle_geometry_;
};

#endif // VEHICLE_FOOTPRINT_HPP
