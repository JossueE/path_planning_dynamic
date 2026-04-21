cmake_minimum_required(VERSION 3.8)
project(path_planning_dynamic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(polygon_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_geometry REQUIRED) 

set(MSG_FILES
  "msg/Obstacle.msg"
  "msg/ObstacleCollection.msg"
  "msg/RoadElements.msg"
  "msg/RoadElementsCollection.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs geometry_msgs polygon_msgs
)

find_package(lanelet2_core)
find_package(lanelet2_io)
find_package(lanelet2_maps)
find_package(lanelet2_projection)
find_package(lanelet2_routing)
find_package(lanelet2_traffic_rules)
find_package(lanelet2_validation)

add_executable(path_planning_node
                src/path_planning.cpp
                src/sat_collision_checker.cc
                src/CarData.cpp
                src/Grid_map.cpp
                src/GlobalPlanner.cpp
               )

add_executable(pointcloud_clustering_node src/pointcloud_clustering.cpp)

add_executable(pointcloud_roi_node src/pointcloud_roi.cpp)

ament_target_dependencies(path_planning_node
  rclcpp
  std_msgs
  sensor_msgs
  grid_map_ros
  geometry_msgs
  visualization_msgs
  tf2
  pcl_conversions
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  OpenCV

  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  polygon_msgs
)

ament_target_dependencies(pointcloud_clustering_node
  rclcpp
  std_msgs
  grid_map_ros
  sensor_msgs
  message_filters
  vision_msgs
  visualization_msgs
  tf2_ros
  geometry_msgs
  pcl_conversions
) 

ament_target_dependencies(pointcloud_roi_node
  rclcpp
  std_msgs
  grid_map_ros
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  visualization_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  polygon_msgs
)

rosidl_target_interfaces(path_planning_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(pointcloud_clustering_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(pointcloud_roi_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


target_include_directories(pointcloud_clustering_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/eigen3
)

target_include_directories(path_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/eigen3
)

target_include_directories(pointcloud_roi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#NOTA: As a result that FastLio forces to use Eigen 3.3.4 or higher, make sure that your system has the correct version installed.

# Install targets
install(TARGETS
  path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_clustering_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_roi_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(path_planning_node
  Eigen3::Eigen
)

target_link_libraries(pointcloud_clustering_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

target_link_libraries(pointcloud_roi_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# Install other files (e.g., launch files)
install(
  DIRECTORY launch src config 
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON for local_path_planning_node.")
message(=============================================================)

ament_package()
