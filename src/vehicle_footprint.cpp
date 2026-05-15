#include "vehicle_footprint.hpp"

#include <cmath>

void VehicleFootprint::setDimensions(double axle_to_front, double axle_to_back,
                                     double width) {
  axle_to_front_ = axle_to_front;
  axle_to_back_ = axle_to_back;
  width_ = width;
  createGeometry();
}

void VehicleFootprint::createGeometry() {
  vehicle_geometry_.points.clear();
  circles_.clear();

  geometry_msgs::msg::Point32 p1;
  geometry_msgs::msg::Point32 p2;
  geometry_msgs::msg::Point32 p3;
  geometry_msgs::msg::Point32 p4;

  p1.x = axle_to_front_;
  p1.y = width_ / 2.0;
  p2.x = axle_to_front_;
  p2.y = -width_ / 2.0;
  p3.x = -axle_to_back_;
  p3.y = -width_ / 2.0;
  p4.x = -axle_to_back_;
  p4.y = width_ / 2.0;

  vehicle_geometry_.points.push_back(p1);
  vehicle_geometry_.points.push_back(p2);
  vehicle_geometry_.points.push_back(p3);
  vehicle_geometry_.points.push_back(p4);
  vehicle_geometry_.points.push_back(p1);

  setCircles();
}

std::vector<Circle> VehicleFootprint::getCircles(const State &state) const {
  std::vector<Circle> result;
  result.reserve(circles_.size());
  const double cos_heading = std::cos(state.heading);
  const double sin_heading = std::sin(state.heading);
  for (const auto &circle : circles_) {
    const double x =
        circle.x * cos_heading - circle.y * sin_heading + state.x;
    const double y =
        circle.x * sin_heading + circle.y * cos_heading + state.y;
    result.emplace_back(x, y, circle.r);
  }
  return result;
}

Circle VehicleFootprint::getBoundingCircle(const State &state) const {
  const double cos_heading = std::cos(state.heading);
  const double sin_heading = std::sin(state.heading);
  const double x = bounding_circle_.x * cos_heading -
                   bounding_circle_.y * sin_heading + state.x;
  const double y = bounding_circle_.x * sin_heading +
                   bounding_circle_.y * cos_heading + state.y;
  return Circle(x, y, bounding_circle_.r);
}

State VehicleFootprint::localToGlobal(const State &reference,
                                      const State &target) const {
  const double x = target.x * std::cos(reference.heading) -
                   target.y * std::sin(reference.heading) + reference.x;
  const double y = target.x * std::sin(reference.heading) +
                   target.y * std::cos(reference.heading) + reference.y;
  const double heading = reference.heading + target.heading;
  return {x, y, heading};
}

visualization_msgs::msg::MarkerArray
VehicleFootprint::toMarkerArray(const std::string &frame_id,
                                const rclcpp::Time &stamp) const {
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "car";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (const auto &pt32 : vehicle_geometry_.points) {
    geometry_msgs::msg::Point pt;
    pt.x = pt32.x;
    pt.y = pt32.y;
    pt.z = 0.0;
    marker.points.push_back(pt);
  }

  markers.markers.push_back(marker);
  return markers;
}

void VehicleFootprint::setCircles() {
  const double small_circle_shift = width_ / 4.0;
  const double small_circle_radius =
      std::hypot(small_circle_shift, small_circle_shift);

  bounding_circle_.x = (axle_to_front_ - axle_to_back_) / 2.0;
  bounding_circle_.y = 0.0;
  bounding_circle_.r = std::hypot((axle_to_front_ + axle_to_back_) / 2.0,
                                  width_ / 2.0);

  circles_.emplace_back(-axle_to_back_ + small_circle_shift, width_ / 2.0,
                        small_circle_radius);
  circles_.emplace_back(-axle_to_back_ + small_circle_shift, -width_ / 2.0,
                        small_circle_radius);
  circles_.emplace_back(axle_to_front_ - small_circle_shift, width_ / 2.0,
                        small_circle_radius);
  circles_.emplace_back(axle_to_front_ - small_circle_shift, -width_ / 2.0,
                        small_circle_radius);
}
