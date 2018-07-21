//
// Created by Murtuza Husain on 21/07/2018.
//

#ifndef GOICPZ_SURFACEREGISTER_H
#define GOICPZ_SURFACEREGISTER_H

#include "goicpzSurfaceUtils.h"
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/sampling_surface_normal.h>

namespace goicpz {

class SurfaceRegister {

public:
    PointCloudT::Ptr cloud;
    PointNormalCloudT::Ptr normals;

    SurfaceRegister():
            cloud(new PointCloudT()),
            normals(new PointNormalCloudT()) {}
    void load_surface(std::string path);

    // Feature selection
    pcl::IndicesPtr select_features(int sample_size);
    pcl::IndicesPtr farthest_point_sample(int num_samples);
    pcl::IndicesPtr normal_space_sample(int num_samples);
    pcl::IndicesPtr normal_space_sample(int num_samples, int bins, int seed);
    pcl::IndicesPtr unique_indices(pcl::IndicesPtr sample1, pcl::IndicesPtr sample2);

    // Descriptors
    std::vector<std::vector<float>> compute_descriptors(pcl::IndicesPtr features);
    std::vector<std::vector<float>> compute_descriptors(pcl::IndicesPtr features, int bins, int radius);

    // Geodesic distances


    void calculate_distances();
    void save();
};

}

#endif //GOICPZ_SURFACEREGISTER_H
