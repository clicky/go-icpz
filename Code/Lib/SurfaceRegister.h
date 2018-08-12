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
#include <flann/flann.hpp>

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

    pcl::IndicesPtr feature_indexes;
    pcl::IndicesPtr feature_boundary_indexes;

    std::vector<std::vector<float>> distances_features;
    std::vector<float> distances_feature_boundary;

    SurfaceRegister():
            surface(new PointCloudT()),
            boundary(new PointCloudT()),
            normals(new PointNormalCloudT()),
            feature_indexes(new std::vector<int>()),
            feature_boundary_indexes(new std::vector<int>())
    {}

public:
    // IO
    void load_surface(std::string surfacePath, std::string boundaryPath);
    void read_ply(std::string path, PointCloudT::Ptr cloud);
    void compute_surface_normals();

    // Feature selection
    pcl::IndicesPtr select_feature_points(PointCloudT::Ptr surface, int sample_size);
    pcl::IndicesPtr sample_features(int sample_size);
    pcl::IndicesPtr farthest_point_sample(int num_samples);
    pcl::IndicesPtr normal_space_sample(int num_samples);
    pcl::IndicesPtr normal_space_sample(int num_samples, int bins, int seed);
    pcl::IndicesPtr remove_boundary_points(PointCloudT::Ptr surface, pcl::IndicesPtr points);
    pcl::IndicesPtr unique_indices(pcl::IndicesPtr sample1, pcl::IndicesPtr sample2);
    PointCloudT::Ptr extract_points(PointCloudT::Ptr mesh, pcl::IndicesPtr feature_indices);

    // Descriptors
    std::vector<std::vector<float>> compute_descriptors(PointCloudT::Ptr surface, pcl::IndicesPtr features);
    std::vector<std::vector<float>> compute_descriptors(PointCloudT::Ptr surface, pcl::IndicesPtr features, int bins, int radius);

    // Geodesic distances
    std::vector<std::vector<float>> compute_distances(PointCloudT::Ptr surface);
    std::vector<float> compute_boundary_distances(PointCloudT::Ptr surface, pcl::IndicesPtr features);

    // Utils
    void save();
    PointCloudT::Ptr getSurface();
    PointCloudT::Ptr getSurfaceBoundary();
    PointNormalCloudT::Ptr getNormals();
    pcl::IndicesPtr getFeatureIndexes();
};

class MovingSurfaceRegister: public SurfaceRegister {

protected:
    PointCloudT::Ptr mask;

public:
    MovingSurfaceRegister():
            mask(new PointCloudT()) {}

    void load_surface(std::string surfacePath, std::string maskPath, std::string boundaryPath);

    PointCloudT::Ptr getMask();
};

class TargetSurfaceRegister: public SurfaceRegister {

protected:
    PointCloudT::Ptr interpolated;

public:
    PointNormalCloudT interpolate_surface();

};

}

#endif //GOICPZ_SURFACEREGISTER_H
