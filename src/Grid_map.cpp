#include "Grid_map.h"

Grid_map::Grid_map(const nav_msgs::msg::OccupancyGrid &map_data)
{
    map_data_ = map_data;
    resolution = map_data.info.resolution;
    originX = map_data.info.origin.position.x;
    originY = map_data.info.origin.position.y;
    width = map_data.info.width;
    height = map_data.info.height;

    // Compute the length of the map in meters
    double length_x = width * resolution;
    double length_y = height * resolution;

    // Compute the center position of the map
    double center_x = originX + length_x / 2.0;
    double center_y = originY + length_y / 2.0;

    // Initialize the GridMap object with "obstacle" and "distance" layers
    map_ = grid_map::GridMap({"obstacle", "distance"});
    map_.setFrameId(map_data.header.frame_id);
    map_.setGeometry(grid_map::Length(length_x, length_y), resolution, grid_map::Position(center_x, center_y));

    // Copy data from the occupancy grid to the "obstacle" layer
    grid_map::Matrix &obstacleData = map_["obstacle"];
    obstacleData.setConstant(map_.getSize()(0), map_.getSize()(1), 1.0);

    cv::Mat obstacle_map(map_.getSize()(1), map_.getSize()(0), CV_8UC1, cv::Scalar(255));

    // Loop through the occupancy grid to populate the obstacle map
    auto mapDataIter = map_data_.data.begin();
    for (unsigned int y = 0; y < map_.getSize()(1); ++y)
    {
        for (unsigned int x = 0; x < map_.getSize()(0); ++x)
        {
            if (*mapDataIter > free_thres_ || *mapDataIter < 0)
            {
                obstacleData(x, y) = 0.0;
                obstacle_map.at<uchar>(y, x) = 0;
            }
            else
            {
                obstacleData(x, y) = 1.0;
                obstacle_map.at<uchar>(y, x) = 255;
            }
            ++mapDataIter;
        }
    }

    // cv::imwrite("obstacle_map.png", obstacle_map);

    // Convert obstacle data to binary image for distance transform
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary = obstacleData.cast<unsigned char>();

    // Convert Eigen matrix to cv::Mat using OpenCV's function
    cv::Mat binary_cv;
    cv::eigen2cv(binary, binary_cv);

    // Compute the distance transform
    cv::Mat distance_cv;
    cv::distanceTransform(binary_cv, distance_cv, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // Flip the distance_cv both vertically and horizontally (rotate 180 degrees)
    cv::flip(distance_cv, distance_cv, -1);

    // Convert cv::Mat to Eigen matrix
    Eigen::MatrixXf distance;
    cv::cv2eigen(distance_cv, distance);

    // Assign the computed distance back to the "distance" layer
    map_["distance"] = distance * resolution;

    // Save the distance map for visualization
    // cv::Mat distance_visual;
    // cv::normalize(distance_cv, distance_visual, 0, 255, cv::NORM_MINMAX);
    // distance_visual.convertTo(distance_visual, CV_8UC1);
    // cv::imwrite("distance_map.png", distance_visual);
}

Grid_map::~Grid_map()
{
}

std::tuple<int, int> Grid_map::toCellID(State state_)
{
    int cell_x = static_cast<int>((state_.x - originX) / resolution);
    int cell_y = static_cast<int>((state_.y - originY) / resolution);

    return std::make_tuple(cell_x, cell_y);
}

void Grid_map::setVehicleFootprint(const VehicleFootprint &vehicle_footprint)
{
    vehicle_footprint_ = vehicle_footprint;
}

double Grid_map::getObstacleDistance(const Eigen::Vector2d &pos) const
{
    if (isInside(pos))
    {
        return map_.atPosition("distance", pos, grid_map::InterpolationMethods::INTER_LINEAR);
    }
    else
    {
        return 0.0; // Return 0.0 if the point is outside the grid map bounds
    }
}

bool Grid_map::isInside(const Eigen::Vector2d &pos) const
{
    return map_.isInside(pos); // Check if the position is inside the map bounds
}

bool Grid_map::isSingleStateCollisionFree(const State &current)
{
    // auto init_time = std::chrono::system_clock::now();

    // Get the vehicle footprint as circles in global coordinates
    std::vector<Circle> footprint = vehicle_footprint_.getCircles(current);

    // Loop through each circle in the footprint
    for (const auto &circle_itr : footprint)
    {
        // Create a position based on the circle's center
        Eigen::Vector2d pos(circle_itr.x, circle_itr.y);

        // Check if the circle is inside the map bounds
        if (isInside(pos))
        {
            // Get the clearance (distance to nearest obstacle)
            double clearance = getObstacleDistance(pos);

            // If the clearance is less than the circle's radius, it means a collision
            if (clearance < circle_itr.r)
            {
                // auto end_time = std::chrono::system_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
                // cout << purple << "--> Execution time check collision new version: " << duration << " ms" << reset << endl;

                return true; // Collision detected
            }
        }
        else
        {
            // auto end_time = std::chrono::system_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
            // cout << purple << "--> Execution time check collision new version: " << duration << " ms" << reset << endl;
            // If out of bounds, consider it a collision
            return true;
        }
    }

    // auto end_time = std::chrono::system_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    // cout << purple << "--> Execution time check collision new version: " << duration << " ms" << reset << endl;

    // No collision detected after checking all circles
    return false;
}

bool Grid_map::isSingleStateCollisionFreeImproved(const State &current)
{
    // Get the bounding circle for the vehicle in global coordinates
    Circle bounding_circle = vehicle_footprint_.getBoundingCircle(current);

    // Create a position based on the bounding circle's center
    Eigen::Vector2d pos(bounding_circle.x, bounding_circle.y);

    // Check if the bounding circle is inside the map bounds
    if (isInside(pos))
    {
        // Get the clearance (distance to nearest obstacle)
        double clearance = getObstacleDistance(pos);

        // If the clearance is less than the radius of the bounding circle, we need to do detailed checks
        if (clearance < bounding_circle.r)
        {
            // Perform detailed collision checking with individual footprint circles
            return isSingleStateCollisionFree(current);
        }
        else
        {
            // No collision if clearance is larger than the bounding circle's radius
            return false;
        }
    }
    else
    {
        // If out of bounds, consider it a collision
        return true;
    }
}
