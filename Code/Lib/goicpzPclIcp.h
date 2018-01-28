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
#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    class PclRegister {
    private:
        // The point clouds we will be using
        PointCloudT::Ptr cloud_in;  // Original point cloud
        PointCloudT::Ptr cloud_tr;  // Transformed point cloud
        PointCloudT::Ptr cloud_icp; // ICP output point cloud
        pcl::console::TicToc time;
    public:
        PclRegister() :
                cloud_in(new PointCloudT),
                cloud_tr(new PointCloudT),
                cloud_icp(new PointCloudT)
        {}
        void registerFixedSurface(std::string path);
        void registerTargetSurface(std::string path);
        void performIcp();
    private:
        void print4x4Matrix(const Eigen::Matrix4d & matrix);
    };

}

#endif //GOICPZ_GOICPZPCLICP_H