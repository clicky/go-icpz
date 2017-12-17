//
// Created by Murtuza Husain on 17/12/2017.
//

#ifndef GOICPZ_GOICPZPCLICP_H
#define GOICPZ_GOICPZPCLICP_H

#include "goicpzPclIcp.h"
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    class PclRegister {
        // The point clouds we will be using
        PointCloudT::Ptr cloud_in;  // Original point cloud
        PointCloudT::Ptr cloud_tr;  // Transformed point cloud
        PointCloudT::Ptr cloud_icp;  // ICP output point cloud
    public:
        PclRegister() {
            cloud_in(new PointCloudT);
            cloud_tr(new PointCloudT);
            cloud_icp(new PointCloudT);
        }
        void registerFixedSurface(const std::str path);
        void applyTransformation();
        void performIcp(int iterations);
        void print4x4Matrix(const Eigen::Matrix4d & matrix);
    };

}

#endif //GOICPZ_GOICPZPCLICP_H