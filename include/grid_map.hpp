#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/opencv.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "vehicle_footprint.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include "state.hpp"
#include <vector>
#include <iostream>
#include <tuple>

using namespace std;

class Grid_map
{
private:
    /* data */
    nav_msgs::msg::OccupancyGrid map_data_;
    double resolution;
    double originX;
    double originY;
    unsigned int width;
    unsigned int height;
    double free_thres_ = 20;

    VehicleFootprint vehicle_footprint_;

    grid_map::GridMap map_;

public:
    Grid_map(const nav_msgs::msg::OccupancyGrid &map_data);
    ~Grid_map();

    void setVehicleFootprint(const VehicleFootprint &vehicle_footprint);
    double getObstacleDistance(const Eigen::Vector2d &pos) const;
    bool isInside(const Eigen::Vector2d &pos) const;
    bool isSingleStateCollisionFree(const State &current);
    bool isSingleStateCollisionFreeImproved(const State &current);

    std::tuple<int, int> toCellID(State start_state);
};

#endif // GRID_MAP_HPP
