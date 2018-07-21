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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointNormal NormalT;
typedef pcl::PointCloud<NormalT> PointNormalCloudT;

namespace goicpz {

class SurfaceRegister {

protected:
    PointCloudT::Ptr surface;
    PointCloudT::Ptr boundary;
    PointNormalCloudT::Ptr normals;

public:
    SurfaceRegister():
            surface(new PointCloudT()),
            boundary(new PointCloudT()),
            normals(new PointNormalCloudT()) {}

    // IO
    void load_surface(std::string mask, std::string boundary);
    void read_ply(std::string path, PointCloudT::Ptr cloud);

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
    void compute_distances();

    // Utils
    void save();
    PointCloudT::Ptr getSurface();
    PointCloudT::Ptr getSurfaceBoundary();
    PointNormalCloudT::Ptr getNormals();
};

}

#endif //GOICPZ_SURFACEREGISTER_H
