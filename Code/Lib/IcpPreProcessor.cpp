//
// Created by Murtuza Husain on 26/03/2018.
//

#include "IcpPreProcessor.h"
#include "TOLDI.h"
#include <math.h>
#include "Sampling.h"

namespace goicpz {

    void IcpPreProcessor::load_moving_surface(std::String path) {
      load_mesh(path, *movingMesh_surface);
    }

    void IcpPreProcessor::load_moving_top(std::String path) {
      load_mesh(path, *movingMesh_top);
    }

    void IcpPreProcessor::load_moving_boundary(std::String path) {
      load_mesh(path, *movingMesh_boundary);
    }

    void load_mesh(std::String path, PointCloudT::Ptr mesh) {
        if (pcl::io::loadPLYFile(path, *mesh) < 0) {
            std::cerr << "Error loading cloud " << path << std::endl;
        }
        std::cout << "Cloud loaded from " << path << std::endl;
    }

    PointCloudT::Ptr IcpPreProcessor::build_features() {
        PointCloudT::Ptr sample = sample_feature_points(*movingMesh_top);

        // Mask points
        pcl::IndicesPtr mask_points = select_points(*movingMesh_surface, sample);
        // Boundary points
        pcl::IndicesPtr boundary_points = select_points(*movingMesh_surface, *movingMesh_boundary);

        // Moving features
        PointCloudT::Ptr moving_features = select_feature_points(*movingMesh_surface, mask_points);
        return moving_features;
    }

    PointCloudT::Ptr select_feature_points(PointCloudT::Ptr mesh, pcl::IndicesPtr indices) {
        PointCloudT::Ptr features(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ExtractIndices<pcl::PointT> extract;
        extract.setInputCloud(mesh);
        extract.setIndices(sample);
        extract.setNegative(false);
        extract.setFilter(*features);

        return features;
    }

    pcl::IndicesPtr select_points(PointClountT::Ptr mesh, PointCloudT::Ptr sample) {
        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(mesh);

        std::vector<int> correspondence_points;
        std::vector<float> correspondence_distances;

        for (int i = 0; i < sample->points.size(); i++) {
            const int K = 1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            kdtree.nearestKSearch(sample->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);

            correspondence_points.push_back(pointIdxNKNSearch[0]);
            correspondence_distances.push_back(pointNKNSquaredDistance[0]);
        }

        return pcl::IndicesPtr(correspondence_points);
    }

    PointCloudT::Ptr sample_feature_points(PointCloudT::Ptr mesh) {
        MeshSampler sampler;

        // Sample
        pcl::IndicesPtr fartest_sample = sampler.farthest_points(mesh, 1500);
        pcl::IndicesPtr normal_sample = sampelr.normal_sampling(mesh, 500);

        // Unique sampled points
        pcl::IndicesPtr sample(*fartest_sample);
        *sample.reserve(*fartest_sample.size() + *normal_sample.size());
        *sample.insert(*fartest_sample.end(), *normal_sample().start(), *normal_sample.end());
        vector<int>::iterator ip;
        ip = std::unique(*sample.begin(), *sample.size());
        *sample.resize(std::distance(*sample.begin(), ip));

        // TODO remove bndy points

        features_idx = sample;

        PointCloudT::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ExtractIndices<pcl::PointT> extract;
        extract.setInputCloud(mesh);
        extract.setIndices(sample);
        extract.setNegative(false);
        extract.setFilter(*filtered);

        return filtered;
    }

    void IcpPreProcessor::build_descriptors() {
      // Descriptors
      compute_toldi_descriptors(*movingMesh_top, features_idx);

      // Geometric distances
      // TODO
    }

    void compute_toldi_descriptors(
      PointCloudT::Ptr mesh,
      pcl::IndicesPtr features
    ) {
      int bins = 10;
      int supportRadius = 20;

      TOLDI_compute(mesh, features, supportRadius, bins, *histograms);
    }

}
