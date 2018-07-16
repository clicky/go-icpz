//
// Created by Murtuza Husain on 26/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_GOICPZSURFACEUTILS_H
#define GOICPZ_SUPERBUILD_GOICPZSURFACEUTILS_H

#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/normal_space.h>
#include <pcl/filters/extract_indices.h>
#include <string>
#include <vector>
//#include "TOLDI.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    void load_mesh(std::string path, PointCloudT::Ptr mesh);

    // Sampling
    std::vector<int> farthest_points(PointCloudT::Ptr mesh, int num_samples);
    pcl::IndicesPtr normal_sampling(PointCloudT::Ptr mesh, int num_samples);
    pcl::IndicesPtr unique_indices(pcl::IndicesPtr sample1, pcl::IndicesPtr sample2);

    // Feature selection
    pcl::IndicesPtr select_points(PointCloudT::Ptr mesh, pcl::IndicesPtr sample);
    PointCloudT::Ptr select_feature_points(PointCloudT::Ptr mesh, pcl::IndicesPtr feature_indices);

    // Descriptors
    std::vector<std::vector<float>> compute_toldi_descriptors(PointCloudT::Ptr mesh, pcl::IndicesPtr features);

}


#endif //GOICPZ_SUPERBUILD_GOICPZSURFACEUTILS_H
