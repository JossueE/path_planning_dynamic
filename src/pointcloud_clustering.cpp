#include "pointcloud_clustering_node.h"

#include <algorithm>
#include <cctype>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

namespace
{
constexpr double kDefaultConcaveAlpha = 0.15;

std::string normalizeHullMode(std::string mode)
{
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return mode;
}

bool buildClosedPolygonFromHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &hull_cloud,
                                geometry_msgs::msg::Polygon &polygon)
{
    if (!hull_cloud || hull_cloud->points.size() < 3)
    {
        return false;
    }

    polygon.points.clear();
    for (const auto &point : hull_cloud->points)
    {
        geometry_msgs::msg::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        polygon.points.push_back(p);
    }

    polygon.points.push_back(polygon.points.front());
    return true;
}
} // namespace

pointcloud_clustering_node::pointcloud_clustering_node(/* args */) : Node("pointcloud_clustering_node")
{
    // Parameters
    this->declare_parameter("GROUND_THRESHOLD", 0.0);
    this->declare_parameter("CLUSTER_THRESH", 0.0);
    this->declare_parameter("CLUSTER_MAX_SIZE", 0);
    this->declare_parameter("CLUSTER_MIN_SIZE", 0);
    this->declare_parameter("CONCAVE_ALPHA", kDefaultConcaveAlpha);
    this->declare_parameter("USE_PCA_BOX", false);
    this->declare_parameter("DISPLACEMENT_THRESH", 0.0);
    this->declare_parameter("IOU_THRESH", 0.0);
    this->declare_parameter("USE_TRACKING", false);
    this->declare_parameter("FRAME_ID", std::string("base_link"));
    this->declare_parameter("HULL_MODE", std::string("convex"));

    // Get parameters
    this->get_parameter("GROUND_THRESHOLD", GROUND_THRESHOLD);
    this->get_parameter("CLUSTER_THRESH", CLUSTER_THRESH);
    this->get_parameter("CLUSTER_MAX_SIZE", CLUSTER_MAX_SIZE);
    this->get_parameter("CLUSTER_MIN_SIZE", CLUSTER_MIN_SIZE);
    this->get_parameter("CONCAVE_ALPHA", CONCAVE_ALPHA);
    this->get_parameter("USE_PCA_BOX", USE_PCA_BOX);
    this->get_parameter("DISPLACEMENT_THRESH", DISPLACEMENT_THRESH);
    this->get_parameter("IOU_THRESH", IOU_THRESH);
    this->get_parameter("USE_TRACKING", USE_TRACKING);
    this->get_parameter("FRAME_ID", FRAME_ID);
    this->get_parameter("HULL_MODE", HULL_MODE);

    HULL_MODE = normalizeHullMode(HULL_MODE);
    if (HULL_MODE != "convex" && HULL_MODE != "concave")
    {
        RCLCPP_WARN(this->get_logger(),
                    "\033[1;31mInvalid HULL_MODE '%s'. Falling back to 'convex'.\033[0m",
                    HULL_MODE.c_str());
        HULL_MODE = "convex";
    }

    if (CONCAVE_ALPHA <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(),
                    "\033[1;31mInvalid CONCAVE_ALPHA %.3f. Falling back to %.2f.\033[0m",
                    CONCAVE_ALPHA, kDefaultConcaveAlpha);
        CONCAVE_ALPHA = kDefaultConcaveAlpha;
    }

    // Create subscriber
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_rotated_notground", 10, std::bind(&pointcloud_clustering_node::pointCloudCallback, this, std::placeholders::_1));
    obstacle_info_publisher_ = this->create_publisher<path_planning_dynamic::msg::ObstacleCollection>("/obstacle_info", 10);

    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar3d_Clustering_node initialized.\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> GROUND_THRESHOLD: %f\033[0m", GROUND_THRESHOLD);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_THRESH: %f\033[0m", CLUSTER_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_MAX_SIZE: %d\033[0m", CLUSTER_MAX_SIZE);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_MIN_SIZE: %d\033[0m", CLUSTER_MIN_SIZE);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> HULL_MODE: %s\033[0m", HULL_MODE.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CONCAVE_ALPHA: %f\033[0m", CONCAVE_ALPHA);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> DISPLACEMENT_THRESH: %f\033[0m", DISPLACEMENT_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> IOU_THRESH: %f\033[0m", IOU_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> USE_TRACKING: %d\033[0m", USE_TRACKING);
}

pointcloud_clustering_node::~pointcloud_clustering_node()
{
}

// Point Cloud callback
void pointcloud_clustering_node::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    //std::cout << yellow << "Received pointcloud_notground " << reset << std::endl;
    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    // Check if the input cloud is empty
    if (input_cloud->empty())
    {
        std::cout << red << "Received empty point cloud" << reset << std::endl;
        imaginaryObstacle();
        return;
    }
    //std::cout << red << "NOT Received empty point cloud" << reset << std::endl;
    try
    {
        auto clusters = obstacle_detector->clustering(input_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

        if (!clusters.empty())
        {
            std::cout << green << "Number of clusters: " << clusters.size() << reset << std::endl;
            if (HULL_MODE == "concave")
            {
                concave_hull(clusters);
            }
            else
            {
                convex_hull(clusters);
            }
        }
        else {
            std::cout << red << "NOT avaliable clusters, check the values" << reset << std::endl;
            imaginaryObstacle();
        }
        
    }
    catch (const std::exception &e)
    {
        std::cout << red << "Error processing point cloud: " << e.what() << reset << std::endl;
    }
}

void pointcloud_clustering_node::convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters)
{

    obstacle_collection.obstacles.clear();

    obstacle_collection.header.stamp = rclcpp::Clock{}.now();
    obstacle_collection.header.frame_id = FRAME_ID;

    int index = 0; // Declare an index variable
    for (auto &cluster : cloud_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(cluster);
        hull.setDimension(2);
        hull.reconstruct(*convexHull);

        if (convexHull->empty())
        {
            std::cout << red << "Convex hull is empty" << reset << std::endl;
            continue;
        }
        if (hull.getDimension() == 2)
        {
            // std::vector<geometry_msgs::msg::Point> hull_points;
            path_planning_dynamic::msg::Obstacle obstacle;
            geometry_msgs::msg::Polygon polygon;
            if (!buildClosedPolygonFromHull(convexHull, polygon))
            {
                std::cout << red << "Convex hull does not have enough vertices" << reset << std::endl;
                continue;
            }

            obstacle.polygon = polygon;
            obstacle.id = index;
            obstacle.type = "NONE";

            obstacle_collection.obstacles.push_back(obstacle);

            index++;
        }
    }
    // Publish the convex hull
    if (!obstacle_collection.obstacles.empty())
    {
        std::cout << yellow << "Convex hull markers published" << reset << std::endl;
        // size of the obstacle collection
        std::cout << yellow << "Obstacle collection size: " << obstacle_collection.obstacles.size() << reset << std::endl;
        obstacle_info_publisher_->publish(obstacle_collection);
    }
    else
    {
        imaginaryObstacle();
    }
}

void pointcloud_clustering_node::concave_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters)
{
    obstacle_collection.obstacles.clear();

    obstacle_collection.header.stamp = rclcpp::Clock{}.now();
    obstacle_collection.header.frame_id = FRAME_ID;

    int index = 0;
    for (auto &cluster : cloud_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr selected_hull(new pcl::PointCloud<pcl::PointXYZ>);
        bool use_convex_fallback = false;

        pcl::ConcaveHull<pcl::PointXYZ> concave_hull_builder;
        concave_hull_builder.setInputCloud(cluster);
        concave_hull_builder.setDimension(2);
        concave_hull_builder.setAlpha(CONCAVE_ALPHA);
        concave_hull_builder.reconstruct(*selected_hull);

        if (selected_hull->empty() || concave_hull_builder.getDimension() != 2 || selected_hull->points.size() < 3)
        {
            use_convex_fallback = true;
        }

        if (use_convex_fallback)
        {
            RCLCPP_WARN(this->get_logger(),
                        "\033[1;31mConcave hull fallback to convex for cluster %d\033[0m",
                        index);

            pcl::ConvexHull<pcl::PointXYZ> convex_hull_builder;
            selected_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
            convex_hull_builder.setInputCloud(cluster);
            convex_hull_builder.setDimension(2);
            convex_hull_builder.reconstruct(*selected_hull);

            if (selected_hull->empty() || convex_hull_builder.getDimension() != 2 || selected_hull->points.size() < 3)
            {
                std::cout << red << "Convex fallback hull is empty or invalid" << reset << std::endl;
                continue;
            }
        }

        path_planning_dynamic::msg::Obstacle obstacle;
        geometry_msgs::msg::Polygon polygon;
        if (!buildClosedPolygonFromHull(selected_hull, polygon))
        {
            std::cout << red << "Selected hull does not have enough vertices" << reset << std::endl;
            continue;
        }

        obstacle.polygon = polygon;
        obstacle.id = index;
        obstacle.type = "NONE";
        obstacle_collection.obstacles.push_back(obstacle);

        index++;
    }

    if (!obstacle_collection.obstacles.empty())
    {
        std::cout << yellow << "Concave hull markers published" << reset << std::endl;
        std::cout << yellow << "Obstacle collection size: " << obstacle_collection.obstacles.size() << reset << std::endl;
        obstacle_info_publisher_->publish(obstacle_collection);
    }
    else
    {
        imaginaryObstacle();
    }
}

void pointcloud_clustering_node::imaginaryObstacle()
{
    // This function can be implemented to create imaginary obstacles for convenience
    // > If clustering dont detect any obstacle, this function can create a static obstacle far from the robot
    // > Or is useful if pointcloud is empty
    obstacle_collection.obstacles.clear();
    obstacle_collection.header.stamp = rclcpp::Clock{}.now();
    obstacle_collection.header.frame_id = "base_link";

    path_planning_dynamic::msg::Obstacle obstacle;
    geometry_msgs::msg::Polygon polygon;
    geometry_msgs::msg::Point32 p;
    p.x = 5000.0;  // <--- 5km in front of the robot
    p.y = 0.0;
    p.z = 0.0;
    polygon.points.push_back(p);
    obstacle.polygon = polygon;
    obstacle.id = 1;
    obstacle.type = "NONE";
    
    // Add the imaginary obstacle to the collection
    obstacle_collection.obstacles.push_back(obstacle);
    std::cout << yellow << "Imaginary Obstacle published " << reset << std::endl;
    obstacle_info_publisher_->publish(obstacle_collection);

}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_clustering_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
