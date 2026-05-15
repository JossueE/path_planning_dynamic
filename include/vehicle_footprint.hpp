#pragma once

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

#include "state.hpp"

/**
 * @brief Vehicle geometry model used for visualization and collision checking.
 *
 * Builds a rectangular footprint from vehicle dimensions and approximates that
 * footprint with circles so Grid_map can test planned states against obstacles.
 *
 * @return --
 * @note Dimensions must be set with setDimensions() before using the generated geometry.
 */
class VehicleFootprint {
public:
  /**
   * @brief Create an empty vehicle footprint.
   *
   * Initializes the object with zero dimensions and empty geometry.
   *
   * @return --
   * @note Call setDimensions() before requesting circles, bounding circles or markers.
  */
  VehicleFootprint() = default;

  /**
   * @brief Set vehicle dimensions and rebuild the footprint geometry.
   *
   * Stores the distance from the reference axle to the front, the distance to the
   * back and the vehicle width, then regenerates the polygon and circle approximation.
   *
   * @param axle_to_front Distance from the vehicle reference axle to the front bumper in meters.
   * @param axle_to_back Distance from the vehicle reference axle to the rear bumper in meters.
   * @param width Vehicle width in meters.
   * @return --
   * @note This function calls createGeometry(), so previous geometry and circles are replaced.
   */
  void setDimensions(double axle_to_front, double axle_to_back, double width);

  /**
   * @brief Get the footprint circles transformed to a vehicle state.
   *
   * Converts each local-frame footprint circle center into world coordinates using
   * the provided vehicle state.
   *
   * @param state Vehicle state used as the transform reference.
   * @return Vector of footprint circles in the same frame as the provided state.
   * @note The circle radii are not changed by the transform.
   */
  [[nodiscard]] std::vector<Circle> getCircles(const State &state) const;

  /**
   * @brief Get the bounding circle transformed to a vehicle state.
   *
   * Converts the local-frame bounding circle center into world coordinates using
   * the provided vehicle state.
   *
   * @param state Vehicle state used as the transform reference.
   * @return Bounding circle in the same frame as the provided state.
   * @note This circle covers the full rectangular footprint.
   */
  [[nodiscard]] Circle getBoundingCircle(const State &state) const;

  /**
   * @brief Convert the local vehicle polygon into RViz markers.
   *
   * Creates a LINE_STRIP marker from the configured vehicle geometry.
   *
   * @param frame_id Frame used in the marker header.
   * @param stamp Timestamp used in the marker header.
   * @return Marker array containing the vehicle footprint visualization.
   * @note The marker represents the local footprint and does not transform it to a vehicle state.
   */
  [[nodiscard]] visualization_msgs::msg::MarkerArray
  toMarkerArray(const std::string &frame_id, const rclcpp::Time &stamp) const;

private:
  /**
   * @brief Build the local-frame rectangular vehicle footprint.
   *
   * Creates the polygon corners from the configured dimensions and updates the
   * circle approximation used for collision checks.
   *
   * @return --
   * @note The polygon is closed by appending the first point again at the end.
   */
  void createGeometry();

  /**
   * @brief Transform a local target state into the reference state frame.
   *
   * Applies a 2D rotation by reference.heading and translation by reference.x/reference.y.
   *
   * @param reference World-frame reference pose.
   * @param target Local-frame target pose.
   * @return Target pose expressed in the reference frame.
   * @note z is not propagated by this helper.
   */
  [[nodiscard]] State localToGlobal(const State &reference,
                                    const State &target) const;

  /**
   * @brief Build the circle approximation of the vehicle footprint.
   *
   * Creates four small footprint circles and one bounding circle from the configured dimensions.
   *
   * @return --
   * @note The generated circles are local-frame primitives centered around the vehicle reference axle.
   */
  void setCircles();

  double axle_to_front_{0.0}; // distance from the center of the front axle to the front of the vehicle
  double axle_to_back_{0.0}; // distance from the center of the front axle to the back of the vehicle
  double width_{0.0}; // distance between the left and right sides of the vehicle

  std::vector<Circle> circles_;
  Circle bounding_circle_;
  geometry_msgs::msg::Polygon vehicle_geometry_;
};
