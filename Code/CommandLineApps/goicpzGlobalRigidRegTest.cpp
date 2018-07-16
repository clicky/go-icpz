//
// Created by Murtuza Husain on 26/03/2018.
//

#include <iostream>
#include <goicpzSurfaceUtils.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace goicpz;

void precompute_moving(PointCloudT::Ptr moving_mesh, PointCloudT::Ptr moving_mask, PointCloudT::Ptr moving_bndy) {
    // Load meshes

    std::string base_path = "/Volumes/dev/Matlab/data/IPCAI2018Data/";

    load_mesh(base_path + "InitialConfigurationSegmentedSurface.ply", moving_mesh);
    load_mesh(base_path + "InitialCT_top.ply", moving_mask);
    load_mesh(base_path + "InitialCT_boundary.ply", moving_bndy);
}

int main (int argc, char** argv) {
    std::cout << "test123" << std::endl;
    PointCloudT::Ptr moving_mesh(new PointCloudT());
    PointCloudT::Ptr moving_mask(new PointCloudT());
    PointCloudT::Ptr moving_bndy(new PointCloudT());

    precompute_moving(moving_mesh, moving_mask, moving_bndy);

    return 0;
}
