//
// Created by Murtuza Husain on 26/03/2018.
//

#include "goicpzSurfaceUtils.h"

namespace goicpz {

    void load_mesh(std::string path, PointCloudT::Ptr mesh) {
        if (pcl::io::loadPLYFile(path, *mesh) < 0) {
            std::cerr << "Error loading cloud " << path << std::endl;
        }
        std::cout << "Cloud loaded from " << path << std::endl;
    }

    std::vector<int> farthest_points(PointCloudT::Ptr mesh, int num_samples) {
        const int maxIter = 10;

        // random initial points
        const int sz = mesh->points.size();
        int point_indexes[num_samples];
        for (int i = 0; i < num_samples; ++i) {
            point_indexes[i] = std::rand() % sz + 1;
        }

        // knn search
        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(mesh);

        int max_distance = 0;

        std::vector<int> max_iter_points;
        const int K = 1;

        for (int iter = 0; iter < maxIter; iter++) {

            std::vector<int> iter_points; // TODO pcl::IndicesPtr ??
            int iter_d = 0;

            for (int i = 0; i < num_samples; i++) {
                int pnt = point_indexes[i];
                PointT searchPoint = mesh->points[pnt];

                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

                iter_points.push_back(pointIdxNKNSearch[0]);
                iter_d += pointNKNSquaredDistance[0];
            }

            if (iter_d > max_distance) {
                max_distance = iter_d;
                max_iter_points = iter_points;
            }
        }

        return max_iter_points;
    }

    // https://github.com/PointCloudLibrary/pcl/blob/master/test/filters/test_sampling.cpp

    pcl::IndicesPtr normal_sampling(PointCloudT::Ptr mesh, int num_samples) {
        int num_bins = 4;

        pcl::NormalSpaceSampling<PointT, PointT> sample;
        sample.setSeed(50);
        sample.setNormals(mesh); //TODO introduce NormalT
        sample.setInputCloud(mesh);
        sample.setSample(num_samples);
        sample.setBins(num_bins, num_bins, num_bins);

        pcl::IndicesPtr indices(new std::vector<int>());
        sample.filter(*indices);

        return indices;
    }

    pcl::IndicesPtr unique_indices(pcl::IndicesPtr sample1, pcl::IndicesPtr sample2) {
        pcl::IndicesPtr sample(*sample1);
        *sample.reserve(*sample1.size() + *sample2.size());
        *sample.insert(*sample1.end(), *sample2().start(), *sample2.end());

        vector<int>::iterator ip;
        ip = std::unique(*sample.begin(), *sample.size());
        *sample.resize(std::distance(*sample.begin(), ip));

        return sample;
    }

    // Feature selection

    pcl::IndicesPtr select_points(PointClountT::Ptr mesh, pcl::IndicesPtr sample) {
        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(mesh);

        std::vector<int> correspondence_points;
        std::vector<float> correspondence_distances;

        for (int i = 0; i < *sample.size(); i++) {
            const int K = 1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            kdtree.nearestKSearch(mesh->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);

            correspondence_points.push_back(pointIdxNKNSearch[0]);
            correspondence_distances.push_back(pointNKNSquaredDistance[0]);
        }

        return pcl::IndicesPtr(correspondence_points);

        // TODO remove bndy points as they are bad descriptors


    }

    PointCloudT::Ptr select_feature_points(PointCloudT::Ptr mesh, pcl::IndicesPtr feature_indices) {
        PointCloudT::Ptr features(new PointCloudT);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(mesh);
        extract.setIndices(sample);
        extract.setNegative(false);
        extract.setFilter(*feature_indices);

        return features;
    }

    std::vector<std::vector<float>> compute_toldi_descriptors(PointCloudT::Ptr mesh, pcl::IndicesPtr features) {
        int bins = 10;
        int supportRadius = 20;

        std::vector<std::vector<float>> histograms;
        TOLDI_compute(mesh, *features, supportRadius, bins, histograms);

        return histograms;
    }

}