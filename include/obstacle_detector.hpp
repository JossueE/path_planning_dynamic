#pragma once

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <vector>

namespace lidar_obstacle_detector
{
    template <typename PointT>
    class ObstacleDetector
    {
    public:
        ObstacleDetector();
        virtual ~ObstacleDetector();

        // Clustering function
        std::vector<typename pcl::PointCloud<PointT>::Ptr>
        clustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const float cluster_tolerance, const int min_size, const int max_size);

    private:
    };

    // constructor:
    template <typename PointT>
    ObstacleDetector<PointT>::ObstacleDetector() {}

    // de-constructor:
    template <typename PointT>
    ObstacleDetector<PointT>::~ObstacleDetector() {}

    template <typename PointT>
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    ObstacleDetector<PointT>::clustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const float cluster_tolerance, const int min_size, const int max_size)
    {
        if (!cloud) {
            std::cout << "[clustering] ⚠ Nube recibida es NULL\n";
        } else {
            std::cout << "[clustering] Nube recibida con " << cloud->size() << " puntos\n";
        }


        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        // Perform euclidean clustering to group detected obstacles
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (const auto &indices : cluster_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            for (int index : indices.indices)
                cluster->points.push_back(cloud->points[index]);

            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
            std::cout << "CLuster Points:" << cluster->points.size() << " puntos\n";

        }

        return clusters;
    }

}
