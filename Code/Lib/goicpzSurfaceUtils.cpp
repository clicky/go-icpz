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




    // Feature selection

    pcl::IndicesPtr select_points(PointCloudT::Ptr mesh, pcl::IndicesPtr sample) {
        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(mesh);

        pcl::IndicesPtr correspondence_points(new std::vector<int>());
        std::vector<float> correspondence_distances;

        for (int i = 0; i < sample->size(); i++) {
            const int K = 1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            kdtree.nearestKSearch(mesh->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);

            correspondence_points->push_back(pointIdxNKNSearch[0]);
            correspondence_distances.push_back(pointNKNSquaredDistance[0]);
        }

        return correspondence_points;

        // TODO remove bndy points as they are bad descriptors

    }

    /**
     * Extract a subset of points from a point cloud.
     *
     * @param mesh to subsample.
     * @param feature_indices of input mesh required.
     * @return subset of input mesh.
     *
     * @see http://pointclouds.org/documentation/tutorials/extract_indices.php
     */
    PointCloudT::Ptr select_feature_points(PointCloudT::Ptr mesh, pcl::IndicesPtr feature_indices) {
        PointCloudT::Ptr features(new PointCloudT);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(mesh);
        extract.setIndices(feature_indices);
        extract.setNegative(false);
        extract.filter(*features);

        return features;
    }

    std::vector<std::vector<float>> compute_toldi_descriptors(PointCloudT::Ptr mesh, pcl::IndicesPtr features) {
        int bins = 10;
        int supportRadius = 20;

        std::vector<std::vector<float>> histograms;
        //TOLDI_compute(mesh, *features, supportRadius, bins, histograms);

        return histograms;
    }

}