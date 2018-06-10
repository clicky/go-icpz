//
// Created by Murtuza Husain on 26/03/2018.
//

#include "IcpPreProcessor.h"
#include <math.h>

namespace goicpz {

    void IcpPreProcessor::loadMesh(std::String path, PointCloudT::Ptr mesh) {
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
        // Sample
        pcl::IndicesPtr fartest_sample = farthest_points(mesh, 1500);
        pcl::IndicesPtr normal_sample = normal_sampling(mesh, 500);

        // Unique sampled points
        pcl::IndicesPtr sample(*fartest_sample);
        *sample.reserve(*fartest_sample.size() + *normal_sample.size());
        *sample.insert(*fartest_sample.end(), *normal_sample().start(), *normal_sample.end());
        vector<int>::iterator ip;
        ip = std::unique(*sample.begin(), *sample.size());
        *sample.resize(std::distance(*sample.begin(), ip));

        PointCloudT::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::ExtractIndices<pcl::PointT> extract;
        extract.setInputCloud(mesh);
        extract.setIndices(sample);
        extract.setNegative(false);
        extract.setFilter(*filtered);

        return filtered;
    }

    /**
     * Iterative algorithm for finding the subset of mesh points that maximises the total minimised pairwise distances.
     *
     * @param mesh to generate sample from.
     * @param num_samples number of samples to generate.
     */
    pcl::IndicesPtr farthest_points(PointCloudT::Ptr mesh, int num_samples) {
        const int maxIter = 10;

        // random initial points
        const int sz = mesh->points.size();
        int point_indexes[num_samples];
        for (int i = 0; i < n; ++i) {
            point_indexes[i] = std::rand() % sz + 1;
        }

        // knn search
        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(mesh);

        pcl::IndicesPtr fartest_points(new std::vector<int>());
        int max_distance = 0;

        const int K = 1;
        for (int iter = 0; iter < max_iter; iter++) {
            pcl::IndicesPtr iter_points(new std::vector<int>());
            int iter_d = 0;

            for (int i = 0; i < n; i++) {
                int pnt = point_indexes[i];
                PointT searchPoint = mesh->points[pnt];

                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

                iter_points.push_back(mesh->points[pointIdxNKNSearch[0]]);
                iter_d += pointNKNSquaredDistance[0];
            }

            if (iter_d > max_distance) {
                max_distance = iter_d;
                fartest_points = iter_points;
            }
        }

        return fartest_points;
    }

    IndicesPtr normal_sampling(PointCloudT::Ptr mesh, int num_samples) {
        int num_bins = 4;

        pcl::NormalSpaceSampling<<PointT, PointT>> sample;
        sample.setSeed(50);
        sample.setNormals(mesh);
        sample.setInputCloud(mesh);
        sample.setSample(num_samples);
        sample.setBins(num_bins, num_bins, num_bins);

        pcl::IndicesPtr indices(new std::vector<int>());
        normal_space_sampling.filter(*indices);

        return indices;
    }

    vector<double> linspace(int min, int max, int count) {
        vector<double> result;
        double interval = (max - min) / count;
        for (double i = 0; i < count; i++) {
            result.push_back(min + i * interval);
        }
        return result;
    }

}