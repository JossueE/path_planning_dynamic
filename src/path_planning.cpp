#include <path_planning.hpp>

#include <chrono>
#include <cctype>
#include <functional>
#include <iostream>

namespace {

double selectConfiguredDouble(const double preferred, const double legacy) {
    return preferred > 0.0 ? preferred : legacy;
}

int selectConfiguredInt(const int preferred, const int legacy) {
    return preferred > 0 ? preferred : legacy;
}

std::string normalizeModelName(std::string model_name) {
    std::transform(model_name.begin(), model_name.end(), model_name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return model_name;
}

} // namespace

path_planning::path_planning() : Node("path_planning"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{
    this->declare_parameter<std::string>("kinematics.model", "ackermann");
    this->declare_parameter<double>("vehicle.axle_to_front", 0.0);
    this->declare_parameter<double>("vehicle.axle_to_back", 0.0);
    this->declare_parameter<double>("vehicle.width", 0.0);
    this->declare_parameter<int>("planner.segment_steps", 0);
    this->declare_parameter<double>("planner.sample_distance", 0.0);
    this->declare_parameter<double>("planner.square_size_m", 1.6);
    this->declare_parameter<double>("planner.safe_clear", 0.2);
    this->declare_parameter<int>("planner.obstacle_inflation_radius_cells", 1);
    this->declare_parameter<int>("planner.branching_factor", 0);
    this->declare_parameter<double>("ackermann.wheelbase", 0.0);
    this->declare_parameter<double>("ackermann.max_steering_angle", 0.0);
    this->declare_parameter<double>("differential.linear_step", 0.0);
    this->declare_parameter<double>("differential.max_angular_step", 0.2);
    this->declare_parameter<int>("differential.angular_samples", 0);
    this->declare_parameter<bool>("differential.include_in_place_rotation", true);

    this->declare_parameter<double>("maxSteerAngle", 0.0);
    this->declare_parameter<double>("wheelBase", 0.0);
    this->declare_parameter<double>("axleToFront", 0.0);
    this->declare_parameter<double>("axleToBack", 0.0);
    this->declare_parameter<double>("width", 0.0);
    this->declare_parameter<int>("pathLength", 0);
    this->declare_parameter<double>("step_car", 0.0);
    this->declare_parameter<int>("tree_depth", 3);
    this->declare_parameter<int>("branching_factor", 5);
    this->declare_parameter<std::string>("map_path", "");
    this->declare_parameter<double>("x_offset", 0.0);
    this->declare_parameter<double>("y_offset", 0.0);
    this->declare_parameter<double>("z_offset", 0.0);
    this->declare_parameter<std::string>("pose_frame", "lidar_link");
    this->declare_parameter<int>("start_lanelet_id", 0);
    this->declare_parameter<int>("end_lanelet_id", 0);

    // Occupancy grid parameters
    this->declare_parameter<double>("global_planner_resolution", 0.20);
    this->declare_parameter<int>("global_planner_close_radius", 0);
    this->declare_parameter<int>("global_planner_close_iters", 0);
    this->declare_parameter<int>("global_planner_outside_value", 100);
    this->declare_parameter<std::string>("global_planner_frame_id", "map");
    this->declare_parameter<std::string>("global_planner_occupancy_output_topic", "occupancy_grid_complete_map");

    std::string configured_model;
    double vehicle_axle_to_front = 0.0;
    double vehicle_axle_to_back = 0.0;
    double vehicle_width = 0.0;
    int configured_segment_steps = 0;
    double configured_sample_distance = 0.0;
    double configured_square_size_m = 1.6;
    double configured_safe_clear = 0.2;
    int configured_obstacle_inflation_radius_cells = 1;
    int configured_branching_factor = 0;
    double configured_ackermann_wheelbase = 0.0;
    double configured_ackermann_max_steering_angle = 0.0;

    this->get_parameter("kinematics.model", configured_model);
    this->get_parameter("vehicle.axle_to_front", vehicle_axle_to_front);
    this->get_parameter("vehicle.axle_to_back", vehicle_axle_to_back);
    this->get_parameter("vehicle.width", vehicle_width);
    this->get_parameter("planner.segment_steps", configured_segment_steps);
    this->get_parameter("planner.sample_distance", configured_sample_distance);
    this->get_parameter("planner.square_size_m", configured_square_size_m);
    this->get_parameter("planner.safe_clear", configured_safe_clear);
    this->get_parameter("planner.obstacle_inflation_radius_cells", configured_obstacle_inflation_radius_cells);
    this->get_parameter("planner.branching_factor", configured_branching_factor);
    this->get_parameter("ackermann.wheelbase", configured_ackermann_wheelbase);
    this->get_parameter("ackermann.max_steering_angle", configured_ackermann_max_steering_angle);
    this->get_parameter("differential.linear_step", differential_linear_step_);
    this->get_parameter("differential.max_angular_step", differential_max_angular_step_);
    this->get_parameter("differential.angular_samples", differential_angular_samples_);
    this->get_parameter("differential.include_in_place_rotation", differential_include_in_place_rotation_);

    double legacy_max_steer_angle = 0.0;
    double legacy_wheelbase = 0.0;
    double legacy_axle_to_front = 0.0;
    double legacy_axle_to_back = 0.0;
    double legacy_width = 0.0;
    int legacy_path_length = 0;
    double legacy_step_car = 0.0;
    int legacy_branching_factor = 0;

    this->get_parameter("maxSteerAngle", legacy_max_steer_angle);
    this->get_parameter("wheelBase", legacy_wheelbase);
    this->get_parameter("axleToFront", legacy_axle_to_front);
    this->get_parameter("axleToBack", legacy_axle_to_back);
    this->get_parameter("width", legacy_width);
    this->get_parameter("pathLength", legacy_path_length);
    this->get_parameter("step_car", legacy_step_car);
    this->get_parameter("tree_depth", tree_depth);
    this->get_parameter("branching_factor", legacy_branching_factor);
    this->get_parameter("map_path", map_path_);
    this->get_parameter("x_offset", x_offset_);
    this->get_parameter("y_offset", y_offset_);
    this->get_parameter("z_offset", z_offset_);
    this->get_parameter("pose_frame", pose_frame_);
    this->get_parameter("start_lanelet_id", start_lanelet_id_);
    this->get_parameter("end_lanelet_id", end_lanelet_id_);

    kinematic_model_name_ = normalizeModelName(configured_model);
    axle_to_front_ = selectConfiguredDouble(vehicle_axle_to_front, legacy_axle_to_front);
    axle_to_back_ = selectConfiguredDouble(vehicle_axle_to_back, legacy_axle_to_back);
    vehicle_width_ = selectConfiguredDouble(vehicle_width, legacy_width);
    pathLength = selectConfiguredInt(configured_segment_steps, legacy_path_length);
    step_car = selectConfiguredDouble(configured_sample_distance, legacy_step_car);
    square_size_m_ = configured_square_size_m > 0.0 ? configured_square_size_m : 1.6;
    SAFE_CLEAR = configured_safe_clear > 0.0 ? configured_safe_clear : 0.2;
    obstacle_inflation_radius_cells_ =
        std::max(0, configured_obstacle_inflation_radius_cells);
    branching_factor = selectConfiguredInt(configured_branching_factor, legacy_branching_factor);
    ackermann_wheelbase_ = selectConfiguredDouble(configured_ackermann_wheelbase, legacy_wheelbase);
    ackermann_max_steering_angle_ =
        selectConfiguredDouble(configured_ackermann_max_steering_angle, legacy_max_steer_angle);
    if (differential_linear_step_ <= 0.0) {
        differential_linear_step_ = step_car;
    }
    if (differential_angular_samples_ <= 0) {
        differential_angular_samples_ = branching_factor;
    }

    // Occupancy grid parameters
    this->get_parameter("global_planner_resolution", global_planner_resolution_);
    this->get_parameter("global_planner_close_radius", global_planner_close_radius_);
    this->get_parameter("global_planner_close_iters", global_planner_close_iters_);
    this->get_parameter("global_planner_outside_value", global_planner_outside_value_);
    this->get_parameter("global_planner_frame_id", global_planner_frame_id_);
    this->get_parameter("global_planner_occupancy_output_topic", global_planner_occupancy_output_topic_);

    obstacle_info_subscription_ = this->create_subscription<path_planning_dynamic::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&path_planning::obstacle_info_callback, this, std::placeholders::_1));

    // publisher for the occupancy grid of the obstacles
    occupancy_grid_pub_test_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacles", 10);
    
    car_analytics_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/car", 10);
    
    real_trajectories_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/none_real_traj", 10);

    real_trajectories_pub_2 = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/real_trajectories_option_2", 10);

    all_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/all_available_paths", 10);

    sdv_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/sdv_trajectory", 10);

    global_planner_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/global_planner", 10);

    global_planner_occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        global_planner_occupancy_output_topic_, 10);

    // -------------> Initialize the shared pointers  <------------
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    car_state_ = std::make_shared<State>();
    grid_map_ = nullptr;
    global_planner_ = std::make_shared<GlobalPlanner>(x_offset_, y_offset_, map_path_, start_lanelet_id_, end_lanelet_id_, global_planner_resolution_, global_planner_close_radius_, global_planner_close_iters_, global_planner_outside_value_, global_planner_frame_id_);

    vehicle_footprint_.setDimensions(axle_to_front_, axle_to_back_, vehicle_width_);
    AckermannKinematicsConfig ackermann_config;
    ackermann_config.wheelbase = ackermann_wheelbase_;
    ackermann_config.max_steering_angle = ackermann_max_steering_angle_;
    ackermann_config.linear_step = step_car;

    DifferentialKinematicsConfig differential_config;
    differential_config.linear_step = differential_linear_step_;
    differential_config.max_angular_step = differential_max_angular_step_;
    differential_config.moving_angular_samples = differential_angular_samples_;
    differential_config.include_in_place_rotation = differential_include_in_place_rotation_;

    kinematic_model_ =
        makeKinematicModel(kinematic_model_name_, ackermann_config, differential_config);
    motion_primitives_ = kinematic_model_->buildMotionPrimitives(branching_factor);

    auto markers = vehicle_footprint_.toMarkerArray("base_footprint", this->get_clock()->now());

    car_analytics_->publish(markers);


    // log out parameters
    RCLCPP_INFO(this->get_logger(), "\033[1;34mkinematics.model: %s\033[0m", kinematic_model_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mackermann.max_steering_angle: %f\033[0m", ackermann_max_steering_angle_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mackermann.wheelbase: %f\033[0m", ackermann_wheelbase_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mvehicle.axle_to_front: %f\033[0m", axle_to_front_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mvehicle.axle_to_back: %f\033[0m", axle_to_back_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mvehicle.width: %f\033[0m", vehicle_width_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mdifferential.linear_step: %f\033[0m", differential_linear_step_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mdifferential.max_angular_step: %f\033[0m", differential_max_angular_step_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mdifferential.angular_samples: %d\033[0m", differential_angular_samples_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpathLength: %d\033[0m", pathLength);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstep_car: %f\033[0m", step_car);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.square_size_m: %f\033[0m", square_size_m_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.safe_clear: %f\033[0m", SAFE_CLEAR);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mplanner.obstacle_inflation_radius_cells: %d\033[0m",
                obstacle_inflation_radius_cells_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mtree_depth: %d\033[0m", tree_depth);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mbranching_factor: %d\033[0m", branching_factor);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mmap_path: %s\033[0m", map_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mx_offset: %f\033[0m", x_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34my_offset: %f\033[0m", y_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mz_offset: %f\033[0m", z_offset_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpose_frame: %s\033[0m", pose_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstart_lanelet_id: %d\033[0m", start_lanelet_id_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mend_lanelet_id: %d\033[0m", end_lanelet_id_);

    all_waypoints_from_global_planner_ = global_planner_->getAllAllWaypointsStruct();
    publishGlobalPlanner();
    if (global_planner_->isOccupancyGridReady())
    {
        global_planner_occupancy_grid_ = global_planner_->getOccupancyGrid();
        publishGlobalPlannerOccupancyGrid();
        global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(global_planner_occupancy_grid_);
    }
}


path_planning::~path_planning()
{
}

// =============================
// get the state of the car
// =============================
void path_planning::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", pose_frame_, tf2::TimePointZero).transform;
        car_state_->x = pose_tf.translation.x;
        car_state_->y = pose_tf.translation.y;
        car_state_->z = pose_tf.translation.z + z_offset_;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_state_->heading = yaw;
        car_state_valid_ = true;
    }
    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
        car_state_valid_ = false;
    }
}

// =============================
// publish the global planner
// =============================
void path_planning::publishGlobalPlanner()
{
    std::cout << green << "Publishing global planner" << reset << std::endl;
    std::cout << green << "Global planner size: " << all_waypoints_from_global_planner_.size() << reset << std::endl;
    global_planner_markers_.markers.clear();
    
    // Clear previous text markers
    visualization_msgs::msg::Marker clear_text;
    clear_text.header.frame_id = "map";
    clear_text.header.stamp = this->now();
    clear_text.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_text.ns = "global_planner_text";
    global_planner_markers_.markers.push_back(clear_text);
    
    for (size_t i = 0; i < all_waypoints_from_global_planner_.size(); i++)
    {
            // Create waypoint marker for main path from the global planner
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "global_planner";
            waypoint_marker.id = i;
            waypoint_marker.type = visualization_msgs::msg::Marker::ARROW;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;

            waypoint_marker.color.a = 0.8;
            
            // Color based on lane_sequence_id
            int seq_id = all_waypoints_from_global_planner_[i].lane_sequence_id;
            switch (seq_id) {
                case 0: // Main path - Blue
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 1: // First neighbor group - Green
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 2: // Second neighbor group - Red
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 3: // Third neighbor group - Yellow
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 4: // Fourth neighbor group - Magenta
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 5: // Fifth neighbor group - Cyan
                    waypoint_marker.color.r = 0.0;
                    waypoint_marker.color.g = 1.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 6: // Sixth neighbor group - Orange
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.5;
                    waypoint_marker.color.b = 0.0;
                    break;
                case 7: // Seventh neighbor group - Purple
                    waypoint_marker.color.r = 0.5;
                    waypoint_marker.color.g = 0.0;
                    waypoint_marker.color.b = 1.0;
                    break;
                case 8: // Eighth neighbor group - Pink
                    waypoint_marker.color.r = 1.0;
                    waypoint_marker.color.g = 0.75;
                    waypoint_marker.color.b = 0.8;
                    break;
                case 9: // Ninth neighbor group - Light Blue
                    waypoint_marker.color.r = 0.5;
                    waypoint_marker.color.g = 0.8;
                    waypoint_marker.color.b = 1.0;
                    break;
                default: // Higher sequence IDs - Cycle through colors
                    {
                        int color_index = seq_id % 10;
                        switch (color_index) {
                            case 0: waypoint_marker.color.r = 0.0; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 1.0; break;
                            case 1: waypoint_marker.color.r = 0.0; waypoint_marker.color.g = 1.0; waypoint_marker.color.b = 0.0; break;
                            case 2: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 0.0; break;
                            case 3: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 1.0; waypoint_marker.color.b = 0.0; break;
                            case 4: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 1.0; break;
                            case 5: waypoint_marker.color.r = 0.0; waypoint_marker.color.g = 1.0; waypoint_marker.color.b = 1.0; break;
                            case 6: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.5; waypoint_marker.color.b = 0.0; break;
                            case 7: waypoint_marker.color.r = 0.5; waypoint_marker.color.g = 0.0; waypoint_marker.color.b = 1.0; break;
                            case 8: waypoint_marker.color.r = 1.0; waypoint_marker.color.g = 0.75; waypoint_marker.color.b = 0.8; break;
                            case 9: waypoint_marker.color.r = 0.5; waypoint_marker.color.g = 0.8; waypoint_marker.color.b = 1.0; break;
                        }
                    }
                    break;
            }

            waypoint_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
            waypoint_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
            waypoint_marker.pose.position.z = 0.0;

            tf2::Quaternion quaternion;
            quaternion.setRPY(0, 0, all_waypoints_from_global_planner_[i].heading);
            waypoint_marker.pose.orientation.x = quaternion.x();
            waypoint_marker.pose.orientation.y = quaternion.y();
            waypoint_marker.pose.orientation.z = quaternion.z();
            waypoint_marker.pose.orientation.w = quaternion.w();

            waypoint_marker.scale.x = 0.6; // Arrow length
            waypoint_marker.scale.y = 0.2; // Arrow width
            waypoint_marker.scale.z = 0.2; // Arrow height

            global_planner_markers_.markers.push_back(waypoint_marker);
            
            // Create text marker for lane_sequence_id
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "global_planner_text";
            text_marker.id = i;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position text above the arrow
            text_marker.pose.position.x = all_waypoints_from_global_planner_[i].x;
            text_marker.pose.position.y = all_waypoints_from_global_planner_[i].y;
            text_marker.pose.position.z = 0.5; // Above the arrow
            
            // Set text content to lane_sequence_id
            text_marker.text = std::to_string(all_waypoints_from_global_planner_[i].lane_sequence_id);
            
            // Text styling
            text_marker.scale.z = 0.3; // Text size
            text_marker.color.a = 1.0;
            text_marker.color.r = 1.0; // White text
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            
            global_planner_markers_.markers.push_back(text_marker);
    }
    global_planner_publisher_->publish(global_planner_markers_);
}

// =============================
// publish the global planner occupancy grid
// =============================
void path_planning::publishGlobalPlannerOccupancyGrid()
{

    std::cout << green << "Publishing global planner occupancy grid" << reset << std::endl;
    std::cout << green << "Grid size: " << global_planner_occupancy_grid_.info.width << "x" 
                << global_planner_occupancy_grid_.info.height << ", resolution: " 
                << global_planner_occupancy_grid_.info.resolution << reset << std::endl;
    
    global_planner_occupancy_grid_publisher_->publish(global_planner_occupancy_grid_);

}

// =============================
// map combination & rescale for put obstacles in the global map
// =============================
void path_planning::obstacle_info_callback(const path_planning_dynamic::msg::ObstacleCollection::SharedPtr msg)
{
    if (!global_map_)
    {
        RCLCPP_ERROR(this->get_logger(), "Global map is not available");
        return;
    }
    if (msg->obstacles.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle collection is empty; planning with base map only");
    }
    else
    {
        std::cout << green << "Obstacles are available and global map is available" << reset << std::endl;
    }
    getCurrentRobotState();
    publishGlobalPlanner();
    RCLCPP_INFO(this->get_logger(), "\033Path planning map combination update.\033[0m");
    map_combination(msg);
}

cv::Mat path_planning::toMat(const nav_msgs::msg::OccupancyGrid &map)
{
    cv::Mat im(map.info.height, map.info.width, CV_8UC1);
    for (size_t i = 0; i < map.data.size(); i++)
    {
        if (map.data[i] == 0)
            im.data[i] = 254; // Free space
        else if (map.data[i] == 100)
            im.data[i] = 0; // Occupied space
        else
            im.data[i] = 205; // Unknown space
    }
    return im;
}

cv::Mat path_planning::rescaleChunk(const cv::Mat &chunk_mat, double scale_factor)
{
    cv::Mat rescaled_chunk;
    cv::resize(chunk_mat, rescaled_chunk, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    return rescaled_chunk;
}

void path_planning::map_combination(const path_planning_dynamic::msg::ObstacleCollection::SharedPtr msg)
{
    if (!car_state_valid_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping map_combination: robot pose not yet available (TF lookup failed).");
        return;
    }

    auto init_time = std::chrono::system_clock::now();
    const auto current_stamp = this->now();
    // clean the rescaled_chunk_
    rescaled_chunk_->data.clear();

    // Compute grid_map_origin position
    State grid_map_origin;

    // Calculate the position based on the car's heading
    grid_map_origin.x = car_state_->x + forward_distance * cos(car_state_->heading);
    grid_map_origin.y = car_state_->y + forward_distance * sin(car_state_->heading);
    grid_map_origin.z = car_state_->z; // Same height as car's position
    grid_map_origin.heading = car_state_->heading;

    // Convert car state to grid coordinates
    int car_x_grid = static_cast<int>((grid_map_origin.x - global_map_->info.origin.position.x) / global_map_->info.resolution);
    int car_y_grid = static_cast<int>((grid_map_origin.y - global_map_->info.origin.position.y) / global_map_->info.resolution);

    // Define chunk boundaries
    int min_x = std::max(0, car_x_grid - chunk_radius);
    int max_x = std::min(static_cast<int>(global_map_->info.width), car_x_grid + chunk_radius);
    int min_y = std::max(0, car_y_grid - chunk_radius);
    int max_y = std::min(static_cast<int>(global_map_->info.height), car_y_grid + chunk_radius);

    // Initialize the chunk grid
    nav_msgs::msg::OccupancyGrid chunk;
    chunk.header = global_map_->header;
    chunk.info.resolution = global_map_->info.resolution;
    chunk.info.width = max_x - min_x;
    chunk.info.height = max_y - min_y;
    chunk.info.origin.position.x = global_map_->info.origin.position.x + min_x * global_map_->info.resolution;
    chunk.info.origin.position.y = global_map_->info.origin.position.y + min_y * global_map_->info.resolution;
    chunk.info.origin.position.z = car_state_->z;
    chunk.info.origin.orientation.w = 1.0;

    chunk.data.resize(chunk.info.width * chunk.info.height, 0);

    for (int y = min_y; y < max_y; ++y)
    {
        for (int x = min_x; x < max_x; ++x)
        {
            int global_index = y * global_map_->info.width + x;
            int local_x = x - min_x;
            int local_y = y - min_y;
            int chunk_index = local_y * chunk.info.width + local_x;

            chunk.data[chunk_index] = global_map_->data[global_index];
        }
    }

    cv::Mat chunk_mat = toMat(chunk);
    cv::Mat rescaled_chunk_mat = rescaleChunk(chunk_mat, scale_factor);

    rescaled_chunk_->header = global_map_->header;
    rescaled_chunk_->info.resolution = 0.2;
    rescaled_chunk_->info.width = rescaled_chunk_mat.cols;
    rescaled_chunk_->info.height = rescaled_chunk_mat.rows;
    rescaled_chunk_->info.origin.position.x = chunk.info.origin.position.x;
    rescaled_chunk_->info.origin.position.y = chunk.info.origin.position.y;
    rescaled_chunk_->info.origin.position.z = car_state_->z ;
    rescaled_chunk_->info.origin.orientation.w = 1.0;

    rescaled_chunk_->data.resize(rescaled_chunk_->info.width * rescaled_chunk_->info.height, 0);

    for (int i = 0; i < rescaled_chunk_mat.rows * rescaled_chunk_mat.cols; i++)
    {
        if (rescaled_chunk_mat.data[i] == 254)
            rescaled_chunk_->data[i] = 0;
        else if (rescaled_chunk_mat.data[i] == 0)
            rescaled_chunk_->data[i] = 100;
        else
            rescaled_chunk_->data[i] = -1;
    }

    const double cos_heading_window = cos(car_state_->heading);
    const double sin_heading_window = sin(car_state_->heading);
    const int car_x_rescaled = static_cast<int>((car_state_->x - rescaled_chunk_->info.origin.position.x) /
                                                rescaled_chunk_->info.resolution);
    const int car_y_rescaled = static_cast<int>((car_state_->y - rescaled_chunk_->info.origin.position.y) /
                                                rescaled_chunk_->info.resolution);
    const double half_size_meters = square_size_m_ / 2.0;

    std::vector<cv::Point> window_polygon;
    window_polygon.reserve(4);
    const std::vector<std::pair<double, double>> corners = {
        {-half_size_meters, -half_size_meters},
        {half_size_meters, -half_size_meters},
        {half_size_meters, half_size_meters},
        {-half_size_meters, half_size_meters}
    };

    for (const auto &corner : corners) {
        const double rotated_x = corner.first * cos_heading_window - corner.second * sin_heading_window;
        const double rotated_y = corner.first * sin_heading_window + corner.second * cos_heading_window;

        const int grid_x =
            car_x_rescaled + static_cast<int>(std::round(rotated_x / rescaled_chunk_->info.resolution));
        const int grid_y =
            car_y_rescaled + static_cast<int>(std::round(rotated_y / rescaled_chunk_->info.resolution));
        window_polygon.emplace_back(grid_x, grid_y);
    }

    cv::Mat window_mask(rescaled_chunk_->info.height, rescaled_chunk_->info.width, CV_8UC1, cv::Scalar(0));
    cv::fillConvexPoly(window_mask, window_polygon, cv::Scalar(255));

    nav_msgs::msg::OccupancyGrid dynamic_obstacle_grid = *rescaled_chunk_;
    dynamic_obstacle_grid.header.stamp = current_stamp;
    dynamic_obstacle_grid.data.assign(dynamic_obstacle_grid.info.width * dynamic_obstacle_grid.info.height, -1);
    for (int y = 0; y < window_mask.rows; ++y)
    {
        for (int x = 0; x < window_mask.cols; ++x)
        {
            if (window_mask.at<uint8_t>(y, x) != 0)
            {
                dynamic_obstacle_grid.data[y * dynamic_obstacle_grid.info.width + x] = 0;
            }
        }
    }

    nav_msgs::msg::OccupancyGrid dynamic_global_obstacle_grid = *global_map_;
    dynamic_global_obstacle_grid.header.stamp = current_stamp;

    auto mark_grid = [&](int x, int y, int value)
    {
        if (x >= 0 && x < static_cast<int>(rescaled_chunk_->info.width) &&
            y >= 0 && y < static_cast<int>(rescaled_chunk_->info.height) &&
            window_mask.at<uint8_t>(y, x) != 0)
        {
            rescaled_chunk_->data[y * rescaled_chunk_->info.width + x] = value; // Mark the cell
            dynamic_obstacle_grid.data[y * dynamic_obstacle_grid.info.width + x] = value;

            const double world_x =
                rescaled_chunk_->info.origin.position.x + (static_cast<double>(x) + 0.5) * rescaled_chunk_->info.resolution;
            const double world_y =
                rescaled_chunk_->info.origin.position.y + (static_cast<double>(y) + 0.5) * rescaled_chunk_->info.resolution;

            const int global_x = static_cast<int>(std::floor(
                (world_x - dynamic_global_obstacle_grid.info.origin.position.x) /
                dynamic_global_obstacle_grid.info.resolution));
            const int global_y = static_cast<int>(std::floor(
                (world_y - dynamic_global_obstacle_grid.info.origin.position.y) /
                dynamic_global_obstacle_grid.info.resolution));

            if (global_x >= 0 && global_x < static_cast<int>(dynamic_global_obstacle_grid.info.width) &&
                global_y >= 0 && global_y < static_cast<int>(dynamic_global_obstacle_grid.info.height))
            {
                dynamic_global_obstacle_grid
                    .data[global_y * dynamic_global_obstacle_grid.info.width + global_x] = value;
            }
        }
    };

    auto inflate_point = [&](int x, int y, int radius, int value)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                if (dx * dx + dy * dy <= radius * radius)
                { // Circle equation
                    mark_grid(x + dx, y + dy, value);
                }
            }
        }
    };

    auto draw_inflated_line = [&](int x0, int y0, int x1, int y1, int radius, int value)
    {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int n = 1 + dx + dy;
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n)
        {
            inflate_point(x0, y0, radius, value);

            if (error > 0)
            {
                x0 += x_inc;
                error -= dy;
            }
            else
            {
                y0 += y_inc;
                error += dx;
            }
        }
    };

    const int inflation_radius = obstacle_inflation_radius_cells_;
    int value_to_mark = 100;

    // Transformation from lidar frame to map frame
    double cos_heading = cos(car_state_->heading);
    double sin_heading = sin(car_state_->heading);

    // Function to check if a point is inside a polygon using ray casting algorithm
    auto point_in_polygon = [&](int x, int y, const std::vector<std::pair<int, int>>& polygon) -> bool
    {
        bool inside = false;
        int j = polygon.size() - 1;
        
        for (int i = 0; i < static_cast<int>(polygon.size()); i++)
        {
            if (((polygon[i].second > y) != (polygon[j].second > y)) &&
                (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
                 (polygon[j].second - polygon[i].second) + polygon[i].first))
            {
                inside = !inside;
            }
            j = i;
        }
        return inside;
    };

    // Function to fill obstacle interiors with dark grid
    auto fill_obstacle_interior = [&](const std::vector<std::pair<int, int>>& polygon_vertices, int fill_value)
    {
        if (polygon_vertices.size() < 3) return; // Need at least 3 points for a polygon
        
        // Find bounding box of the polygon
        int min_x = polygon_vertices[0].first, max_x = polygon_vertices[0].first;
        int min_y = polygon_vertices[0].second, max_y = polygon_vertices[0].second;
        
        for (const auto& vertex : polygon_vertices)
        {
            min_x = std::min(min_x, vertex.first);
            max_x = std::max(max_x, vertex.first);
            min_y = std::min(min_y, vertex.second);
            max_y = std::max(max_y, vertex.second);
        }
        
        // Clamp to grid boundaries
        min_x = std::max(0, min_x);
        max_x = std::min(static_cast<int>(rescaled_chunk_->info.width) - 1, max_x);
        min_y = std::max(0, min_y);
        max_y = std::min(static_cast<int>(rescaled_chunk_->info.height) - 1, max_y);
        
        // Fill all points inside the polygon
        for (int y = min_y; y <= max_y; ++y)
        {
            for (int x = min_x; x <= max_x; ++x)
            {
                if (point_in_polygon(x, y, polygon_vertices))
                {
                    mark_grid(x, y, fill_value);
                }
            }
        }
    };

    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        const auto &obstacle = msg->obstacles[i];
        
        // Store polygon vertices in grid coordinates for filling
        std::vector<std::pair<int, int>> polygon_vertices;
        
        for (size_t j = 0; j < obstacle.polygon.points.size(); ++j)
        {
            auto &current_point_lidar = obstacle.polygon.points[j];
            auto &next_point_lidar = obstacle.polygon.points[(j + 1) % obstacle.polygon.points.size()];

            geometry_msgs::msg::Point current_point_map;
            current_point_map.x = car_state_->x + cos_heading * current_point_lidar.x - sin_heading * current_point_lidar.y;
            current_point_map.y = car_state_->y + sin_heading * current_point_lidar.x + cos_heading * current_point_lidar.y;
            geometry_msgs::msg::Point next_point_map;
            next_point_map.x = car_state_->x + cos_heading * next_point_lidar.x - sin_heading * next_point_lidar.y;
            next_point_map.y = car_state_->y + sin_heading * next_point_lidar.x + cos_heading * next_point_lidar.y;

            int x0 = static_cast<int>((current_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y0 = static_cast<int>((current_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);
            int x1 = static_cast<int>((next_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y1 = static_cast<int>((next_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);

            // Store vertex for polygon filling
            polygon_vertices.push_back({x0, y0});

            // Draw inflated boundary
            draw_inflated_line(x0, y0, x1, y1, inflation_radius, value_to_mark);
        }
        
        // Fill the interior of the obstacle with dark grid (value 100)
        fill_obstacle_interior(polygon_vertices, value_to_mark);
    }

    nav_msgs::msg::OccupancyGrid published_dynamic_obstacle_grid = dynamic_obstacle_grid;
    cv::Rect window_bounds = cv::boundingRect(window_polygon);
    window_bounds &= cv::Rect(0, 0,
                              static_cast<int>(dynamic_obstacle_grid.info.width),
                              static_cast<int>(dynamic_obstacle_grid.info.height));
    if (window_bounds.width > 0 && window_bounds.height > 0)
    {
        published_dynamic_obstacle_grid.info.width = static_cast<uint32_t>(window_bounds.width);
        published_dynamic_obstacle_grid.info.height = static_cast<uint32_t>(window_bounds.height);
        published_dynamic_obstacle_grid.info.origin.position.x +=
            static_cast<double>(window_bounds.x) * dynamic_obstacle_grid.info.resolution;
        published_dynamic_obstacle_grid.info.origin.position.y +=
            static_cast<double>(window_bounds.y) * dynamic_obstacle_grid.info.resolution;
        published_dynamic_obstacle_grid.data.assign(
            published_dynamic_obstacle_grid.info.width * published_dynamic_obstacle_grid.info.height, -1);

        for (int y = 0; y < window_bounds.height; ++y)
        {
            for (int x = 0; x < window_bounds.width; ++x)
            {
                const int src_x = window_bounds.x + x;
                const int src_y = window_bounds.y + y;
                published_dynamic_obstacle_grid.data[
                    y * published_dynamic_obstacle_grid.info.width + x] =
                    dynamic_obstacle_grid.data[src_y * dynamic_obstacle_grid.info.width + src_x];
            }
        }
    }

    buildDistanceField();
    buildWaypointDistanceFields();
    grid_map_ = std::make_shared<Grid_map>(*rescaled_chunk_);
    grid_map_->setVehicleFootprint(vehicle_footprint_);

    global_planner_occupancy_grid_ = dynamic_global_obstacle_grid;
    occupancy_grid_pub_test_->publish(published_dynamic_obstacle_grid);
    global_planner_occupancy_grid_publisher_->publish(global_planner_occupancy_grid_);

    TreeFlat flat_map;
    int best_map = generateTrajectoryTree_AStar_flat_map_with_waypoints(*car_state_, flat_map);
    publishBestPathFromFlat(flat_map, best_map, 2); // blue color for the A* implementation with waypoints
    publishAllPathsFromFlat(flat_map); // publish all available paths
    publishTrajectoryPath(flat_map, best_map); // publish chosen trajectory as nav_msgs::Path


    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    std::cout << blue << "Execution time for path selection: " << duration << " ms" << reset << std::endl;
}

  
// =============================
// generate the trajectory based on the A* algorithm
// =============================
int path_planning::generateTrajectoryTree_AStar_flat_map_with_waypoints(const State& root_state, TreeFlat& out)
{
    return generateTrajectoryTreeImpl(root_state, out, true);
}

int path_planning::generateTrajectoryTreeImpl(const State& root_state, TreeFlat& out, const bool use_waypoints)
{
    out.nodes.clear();
    out.leaves.clear();

    const int B = std::max(1, static_cast<int>(motion_primitives_.size()));
    const int D = std::max(0, tree_depth);
    const int EFFECTIVE_DEPTH = (D > 0) ? (D - 1) : 0;

    const double cs0 = std::cos(root_state.heading);
    const double ss0 = std::sin(root_state.heading);

    size_t max_nodes = 1;
    size_t powB = 1;
    for (int d = 0; d < EFFECTIVE_DEPTH; ++d) {
        powB *= static_cast<size_t>(B);
        max_nodes += powB;
    }
    max_nodes = std::min(max_nodes, static_cast<size_t>(500000));
    out.nodes.reserve(max_nodes);

    FlatNode root;
    root.state = root_state;
    root.parent = -1;
    root.primitive_index = -1;
    root.depth = 0;
    root.cost = 0.0;
    out.nodes.push_back(root);

    int best_goal_idx = -1;
    double best_goal_cost = std::numeric_limits<double>::infinity();

    std::priority_queue<PQItem> open;
    std::unordered_map<LatticeKey, double, LatticeKeyHash> best_g;

    auto stateKey = [&](const State& s) -> LatticeKey {
        return LatticeKey{static_cast<int>(s.gridx), static_cast<int>(s.gridy), heading_bin(s.heading)};
    };

    auto h_lower_bound = [&](int depth) -> double {
        const int remaining_segments = EFFECTIVE_DEPTH - depth;
        if (remaining_segments <= 0) {
            return 0.0;
        }
        const int remaining_steps = remaining_segments * pathLength;
        return -W_FORWARD * (remaining_steps * kinematic_model_->maxForwardStep());
    };

    auto sample_wp_dist = [&](int gx, int gy) -> double {
        if (!use_waypoints) {
            return 0.0;
        }
        if (has_wp1_ && !dist_wp1_m_.empty()) {
            if (gy >= 0 && gy < dist_wp1_m_.rows && gx >= 0 && gx < dist_wp1_m_.cols) {
                return static_cast<double>(dist_wp1_m_.at<float>(gy, gx)) * W_WP1;
            }
            return 0.0;
        }
        if (has_wp2_ && !dist_wp2_m_.empty()) {
            if (gy >= 0 && gy < dist_wp2_m_.rows && gx >= 0 && gx < dist_wp2_m_.cols) {
                return static_cast<double>(dist_wp2_m_.at<float>(gy, gx)) * W_WP2;
            }
            return 0.0;
        }
        return 0.0;
    };

    {
        const LatticeKey k = stateKey(root.state);
        best_g[k] = 0.0;
        open.push(PQItem{0, h_lower_bound(0), 0.0});
    }

    auto expand_one = [&](int parent_idx, size_t primitive_idx) -> int {
        const FlatNode& parent = out.nodes[parent_idx];
        const MotionPrimitive& primitive = motion_primitives_[primitive_idx];
        RolloutResult rollout =
            kinematic_model_->rollout(parent.state, primitive, pathLength);

        if (rollout.samples.empty()) {
            return -1;
        }

        double obs_pen_sum = 0.0;
        double wp_pen_sum = 0.0;

        for (auto& ns : rollout.samples) {
            ns.z = parent.state.z;
            auto cell = grid_map_->toCellID(ns);
            ns.gridx = std::get<0>(cell);
            ns.gridy = std::get<1>(cell);

            if (grid_map_->isSingleStateCollisionFreeImproved(ns)) {
              return -1;
            }

            const double d = clearanceMeters(static_cast<int>(ns.gridx), static_cast<int>(ns.gridy));
            if (d < SAFE_CLEAR) {
                obs_pen_sum += (SAFE_CLEAR - d);
            }

            if (use_waypoints && d < SAFE_CLEAR * 0.6) {
                return -1;
            }

            wp_pen_sum += sample_wp_dist(static_cast<int>(ns.gridx), static_cast<int>(ns.gridy));
        }

        FlatNode child;
        child.state = rollout.samples.back();
        child.parent = parent_idx;
        child.primitive_index = static_cast<int>(primitive_idx);
        child.depth = parent.depth + 1;
        child.segment_samples = std::move(rollout.samples);

        const MotionPrimitive* previous_primitive =
            parent.primitive_index >= 0 ? &motion_primitives_[parent.primitive_index] : nullptr;
        const double steer_pen =
            W_STEER * kinematic_model_->controlEffort(primitive);
        const double dsteer_pen =
            W_DSTEER * kinematic_model_->smoothnessCost(previous_primitive, primitive);

        const double dx = child.state.x - parent.state.x;
        const double dy = child.state.y - parent.state.y;
        const double forward_inc = dx * cs0 + dy * ss0;

        const double obs_pen = W_OBS * (obs_pen_sum / std::max(1, pathLength));
        const double wp_pen =
            use_waypoints ? (wp_pen_sum / std::max(1, pathLength)) : 0.0;

        double straight_penalty = 0.0;
        if (use_waypoints && std::fabs(primitive.angular_step) < 1e-6 &&
            std::fabs(primitive.steering_angle) < 1e-6 && forward_inc > 0.0) {
            const double avg_clearance = obs_pen_sum / std::max(1, pathLength);
            if (avg_clearance > 0.1) {
                straight_penalty = 2.0;
            }
        }

        const double g_child = parent.cost + steer_pen + dsteer_pen + obs_pen +
                               wp_pen + straight_penalty -
                               W_FORWARD * forward_inc;

        child.cost = g_child;

        const LatticeKey ck = stateKey(child.state);
        auto it = best_g.find(ck);
        if (it != best_g.end() && g_child >= it->second - 1e-12) {
            return -1;
        }
        best_g[ck] = g_child;

        out.nodes.push_back(std::move(child));
        return static_cast<int>(out.nodes.size()) - 1;
    };

    while (!open.empty()) {
        const PQItem cur = open.top();
        open.pop();

        const int idx = cur.idx;
        const auto& fn = out.nodes[idx];
        const double g = fn.cost;
        const int d = fn.depth;

        if (std::fabs(g - cur.g_copy) > 1e-12) {
            continue;
        }

        if (d == EFFECTIVE_DEPTH) {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err =
                std::fabs(wrapAngle(fn.state.heading - root_state.heading));

            const double total =
                g + W_LAT * std::fabs(lateral) + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx = idx;
            }

            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) {
                break;
            }
            continue;
        }

        bool produced_child = false;
        for (size_t primitive_idx = 0; primitive_idx < motion_primitives_.size();
             ++primitive_idx) {
            const int child_idx = expand_one(idx, primitive_idx);
            if (child_idx < 0) {
                continue;
            }
            produced_child = true;

            const auto& ch = out.nodes[child_idx];
            open.push(PQItem{child_idx, ch.cost + h_lower_bound(ch.depth),
                             ch.cost});
        }

        if (!produced_child) {
            const double dx = fn.state.x - root_state.x;
            const double dy = fn.state.y - root_state.y;
            const double lateral = -dx * ss0 + dy * cs0;
            const double head_err =
                std::fabs(wrapAngle(fn.state.heading - root_state.heading));

            const double total =
                g + W_LAT * std::fabs(lateral) + W_HEAD * head_err;

            if (total < best_goal_cost) {
                best_goal_cost = total;
                best_goal_idx = idx;
            }
            if (!open.empty() && open.top().f_est >= best_goal_cost - 1e-12) {
                break;
            }
        }
    }

    out.leaves.clear();
    for (size_t i = 0; i < out.nodes.size(); ++i) {
        if (out.nodes[i].depth == EFFECTIVE_DEPTH) {
            out.leaves.push_back(static_cast<int>(i));
        }
    }

    if (best_goal_idx >= 0) {
        const bool is_listed = std::find(out.leaves.begin(), out.leaves.end(),
                                         best_goal_idx) != out.leaves.end();
        if (!is_listed) {
            out.leaves.push_back(best_goal_idx);
        }
    }

    return best_goal_idx;
}


// =============================
//  helper functions for the A* algorithm
// =============================
inline void path_planning::build_chain_indices(
    const TreeFlat& flat, int leaf_idx, std::vector<int>& chain) const
{
    chain.clear();
    for (int i = leaf_idx; i != -1; i = flat.nodes[i].parent) chain.push_back(i);
    std::reverse(chain.begin(), chain.end()); // root -> leaf
}

void path_planning::buildDistanceField()
{
    if (!rescaled_chunk_ || rescaled_chunk_->data.empty()) {
        dist_m_.release();
        return;
    }

    const int H = static_cast<int>(rescaled_chunk_->info.height);
    const int W = static_cast<int>(rescaled_chunk_->info.width);
    const double res = rescaled_chunk_->info.resolution; // 0.2 in your setup

    // Build binary image for distance transform: free=255, else=0 (occupied OR unknown)
    cv::Mat bin(H, W, CV_8UC1);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const int8_t v = rescaled_chunk_->data[y * W + x];
            // Your convention: 0=free, 100=occupied, -1=unknown
            bin.at<uint8_t>(y, x) = (v == 0) ? 255 : 0;
        }
    }

    // Distance transform in pixels
    cv::Mat dist_px;
    cv::distanceTransform(bin, dist_px, cv::DIST_L2, 3);

    // Convert pixels to meters
    dist_m_.create(H, W, CV_32FC1);
    const float scale = static_cast<float>(res);
    for (int y = 0; y < H; ++y) {
        const float* src = dist_px.ptr<float>(y);
        float*       dst = dist_m_.ptr<float>(y);
        for (int x = 0; x < W; ++x) dst[x] = src[x] * scale;
    }
}

inline double path_planning::clearanceMeters(int gx, int gy) const
{
    if (dist_m_.empty()) return 0.0;
    if (gy < 0 || gy >= dist_m_.rows || gx < 0 || gx >= dist_m_.cols) return 0.0;
    return static_cast<double>(dist_m_.at<float>(gy, gx));
}

void path_planning::buildWaypointDistanceFields()
{
    dist_wp1_m_.release();
    dist_wp2_m_.release();
    has_wp1_ = false;
    has_wp2_ = false;

    if (!rescaled_chunk_ || rescaled_chunk_->data.empty()) return;

    const int H = static_cast<int>(rescaled_chunk_->info.height);
    const int W = static_cast<int>(rescaled_chunk_->info.width);
    const double res = rescaled_chunk_->info.resolution;

    // Binary canvases for each priority: 255 where path pixels live, 0 elsewhere
    cv::Mat bin1(H, W, CV_8UC1, cv::Scalar(0));
    cv::Mat bin2(H, W, CV_8UC1, cv::Scalar(0));

    auto worldToGrid = [&](double x, double y, int& gx, int& gy) {
        gx = static_cast<int>((x - rescaled_chunk_->info.origin.position.x) / res);
        gy = static_cast<int>((y - rescaled_chunk_->info.origin.position.y) / res);
    };

    // Helper to draw a thick line in grid space
    auto drawThickLine = [&](cv::Mat& img, int x0, int y0, int x1, int y1, int radius) {
        cv::LineIterator it(img, cv::Point(x0, y0), cv::Point(x1, y1));
        for (int i = 0; i < it.count; ++i, ++it) {
            cv::circle(img, it.pos(), radius, cv::Scalar(255), cv::FILLED);
        }
    };

    // Group consecutive waypoints by priority and draw lines between neighbors
    auto drawByPriority = [&](int prio, cv::Mat& canvas, bool& has_any) {
        std::vector<cv::Point> pts;
        pts.reserve(all_waypoints_from_global_planner_.size());

        // Collect all in-chunk points for this priority
        for (const auto& p : all_waypoints_from_global_planner_) {
            if (p.priority != prio) continue;
            int gx, gy;
            worldToGrid(p.x, p.y, gx, gy);
            if (gx >= 0 && gx < W && gy >= 0 && gy < H) {
                pts.emplace_back(gx, gy);
            }
        }

        // Draw segments between consecutive in-chunk points
        if (pts.size() >= 1) {
            has_any = true;
            const int rad = std::max(1, (int)std::round(WP_STROKE_RADIUS_CELLS));
            // Densify by connecting consecutive points
            for (size_t i = 1; i < pts.size(); ++i) {
                drawThickLine(canvas, pts[i-1].x, pts[i-1].y, pts[i].x, pts[i].y, rad);
            }
            // Also mark isolated singletons
            for (const auto& q : pts) {
                cv::circle(canvas, q, rad, cv::Scalar(255), cv::FILLED);
            }
        }
    };

    drawByPriority(1, bin1, has_wp1_);
    drawByPriority(2, bin2, has_wp2_);

    // If there is no prio-1 in chunk, keep bin1 empty; same for prio-2.
    auto toMetersDist = [&](const cv::Mat& bin, cv::Mat& out_m) {
        if (cv::countNonZero(bin) == 0) {
            out_m.release(); // no geometry to attract to
            return;
        }
        cv::Mat inv; // distanceTransform wants non-zero free area as source; we want distance TO the path
        // Build "feature" mask: 255 at path pixels, 0 elsewhere; we want distance TO those features,
        // so invert to a mask where features are 0 and background is 255, then DT that.
        cv::Mat feat = 255 - bin;

        cv::Mat dist_px;
        cv::distanceTransform(feat, dist_px, cv::DIST_L2, 3);

        out_m.create(bin.rows, bin.cols, CV_32FC1);
        const float scale = static_cast<float>(res);
        for (int y = 0; y < bin.rows; ++y) {
            const float* src = dist_px.ptr<float>(y);
            float*       dst = out_m.ptr<float>(y);
            for (int x = 0; x < bin.cols; ++x) dst[x] = src[x] * scale;
        }
    };

    toMetersDist(bin1, dist_wp1_m_);
    toMetersDist(bin2, dist_wp2_m_);

    // Reconcile availability flags with actual outputs
    has_wp1_ = has_wp1_ && !dist_wp1_m_.empty();
    has_wp2_ = has_wp2_ && !dist_wp2_m_.empty();
}


// =============================
// publish the trajectory
// =============================
void path_planning::publishBestPathFromFlat(const TreeFlat& flat, int leaf_idx, int color_idx)
{
    if (color_idx == 1) 
    {
        if (leaf_idx < 0 || real_trajectories_pub_->get_subscription_count() == 0) return;
    }
    else if (color_idx == 2)
    {
        if (leaf_idx < 0 || real_trajectories_pub_2->get_subscription_count() == 0) return;
    }


    // Build chain root->leaf
    std::vector<int> chain;
    build_chain_indices(flat, leaf_idx, chain);

    visualization_msgs::msg::MarkerArray msg;

    // clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "real_trajectories"; msg.markers.push_back(clear);
    clear.ns = "real_endpoints";     msg.markers.push_back(clear);
    clear.ns = "real_trajectory_labels"; msg.markers.push_back(clear);

    // line marker
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = this->now();
    line.ns = "real_trajectories";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.1;

    // chose btw to color to know wich implementation is used
    if (color_idx == 0) {
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0;
    } else if (color_idx == 1) {
        line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0;
    } else if (color_idx == 2) {
        line.color.r = 0.0; line.color.g = 0.0; line.color.b = 1.0;
    }
    else {
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0;
    }

    line.color.a = 1.0;

    // start at the true root pose so the polyline includes the origin point
    const State &root_state = flat.nodes[chain.front()].state;
    {
        geometry_msgs::msg::Point p;
        p.x = root_state.x; p.y = root_state.y; p.z = root_state.z;
        line.points.push_back(p);
    }

    State end_state = root_state;
    for (size_t k = 1; k < chain.size(); ++k)
    {
        const auto& fn = flat.nodes[chain[k]];
        for (const auto& sample : fn.segment_samples) {
            geometry_msgs::msg::Point p;
            p.x = sample.x;
            p.y = sample.y;
            p.z = sample.z;
            line.points.push_back(p);
            end_state = sample;
        }
    }

    msg.markers.push_back(line);

    // endpoint sphere
    visualization_msgs::msg::Marker endpoint;
    endpoint.header.frame_id = "map";
    endpoint.header.stamp = this->now();
    endpoint.ns = "real_endpoints";
    endpoint.id = 1;
    endpoint.type = visualization_msgs::msg::Marker::SPHERE;
    endpoint.action = visualization_msgs::msg::Marker::ADD;
    endpoint.pose.position.x = end_state.x;
    endpoint.pose.position.y = end_state.y;
    endpoint.pose.position.z = end_state.z + 0.2;
    endpoint.scale.x = 0.2; endpoint.scale.y = 0.2; endpoint.scale.z = 0.2;
    endpoint.color = line.color; endpoint.color.a = 0.8;
    msg.markers.push_back(endpoint);

    if (color_idx == 1) 
    {
        real_trajectories_pub_->publish(msg);
    }
    else if (color_idx == 2)
    {
        real_trajectories_pub_2->publish(msg);
    }
}


void path_planning::publishAllPathsFromFlat(const TreeFlat& flat)
{
    if (all_paths_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray msg;

    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.ns = "all_paths";
    msg.markers.push_back(clear);

    // Publish all paths from root to each leaf
    for (size_t leaf_idx = 0; leaf_idx < flat.leaves.size(); ++leaf_idx)
    {
        int leaf = flat.leaves[leaf_idx];
        if (leaf < 0) continue;

        // Build chain root->leaf
        std::vector<int> chain;
        build_chain_indices(flat, leaf, chain);

        // Create line marker for this path
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now();
        line.ns = "all_paths";
        line.id = static_cast<int>(leaf_idx);
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05; // Thinner lines for all paths

        // Color based on path index (cycle through colors)
        int color_idx = leaf_idx % 6;
        switch (color_idx) {
            case 0: line.color.r = 1.0; line.color.g = 0.0; line.color.b = 0.0; break; // Red
            case 1: line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; break; // Green
            case 2: line.color.r = 0.0; line.color.g = 0.0; line.color.b = 1.0; break; // Blue
            case 3: line.color.r = 1.0; line.color.g = 1.0; line.color.b = 0.0; break; // Yellow
            case 4: line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0; break; // Magenta
            case 5: line.color.r = 0.0; line.color.g = 1.0; line.color.b = 1.0; break; // Cyan
        }
        line.color.a = 0.7; // Semi-transparent

        // Start at the root pose
        const State &root_state = flat.nodes[chain.front()].state;
        {
            geometry_msgs::msg::Point p;
            p.x = root_state.x; p.y = root_state.y; p.z = root_state.z;
            line.points.push_back(p);
        }

        for (size_t k = 1; k < chain.size(); ++k)
        {
            const auto& fn = flat.nodes[chain[k]];
            for (const auto& sample : fn.segment_samples) {
                geometry_msgs::msg::Point p;
                p.x = sample.x;
                p.y = sample.y;
                p.z = sample.z + 0.2;
                line.points.push_back(p);
            }
        }

        msg.markers.push_back(line);
    }

    all_paths_pub_->publish(msg);
}


void path_planning::publishTrajectoryPath(const TreeFlat& flat, int leaf_idx)
{
    if (sdv_trajectory_pub_->get_subscription_count() == 0) return;
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    if (leaf_idx < 0) {
        sdv_trajectory_pub_->publish(path_msg);
        return;
    }

    // Build chain root->leaf
    std::vector<int> chain;
    build_chain_indices(flat, leaf_idx, chain);

    // Start at the root pose
    const State &root_state = flat.nodes[chain.front()].state;
    
    // Add the starting point
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = path_msg.header.stamp;
    start_pose.pose.position.x = root_state.x;
    start_pose.pose.position.y = root_state.y;
    start_pose.pose.position.z = root_state.z;
    
    // Convert heading to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, root_state.heading);
    start_pose.pose.orientation.x = q.x();
    start_pose.pose.orientation.y = q.y();
    start_pose.pose.orientation.z = q.z();
    start_pose.pose.orientation.w = q.w();
    
    path_msg.poses.push_back(start_pose);

    for (size_t k = 1; k < chain.size(); ++k)
    {
        const auto& fn = flat.nodes[chain[k]];
        for (const auto& sample : fn.segment_samples) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = path_msg.header.stamp;
            pose.pose.position.x = sample.x;
            pose.pose.position.y = sample.y;
            pose.pose.position.z = sample.z + 0.2;

            tf2::Quaternion sample_q;
            sample_q.setRPY(0, 0, sample.heading);
            pose.pose.orientation.x = sample_q.x();
            pose.pose.orientation.y = sample_q.y();
            pose.pose.orientation.z = sample_q.z();
            pose.pose.orientation.w = sample_q.w();

            path_msg.poses.push_back(pose);
        }
    }

    sdv_trajectory_pub_->publish(path_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
