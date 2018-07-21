//
// Created by Murtuza Husain on 21/07/2018.
//

#include <pcl/features/normal_3d.h>
#include "SurfaceRegister.h"

namespace goicpz {

    void SurfaceRegister::load_surface(std::string path) {
        load_mesh(path, cloud);

        pcl::NormalEstimation<PointT,NormalT> ne;
        ne.setInputCloud (cloud);
        ne.setRadiusSearch (0.05);
        ne.compute(*normals);
        copyPointCloud(*cloud, *normals);
    }

    /**
     * Sample the point cloud surface using a combination of farthest point and normal space sampling, returning the
     * unique intersection of the two. A 1:3 split is used for farthest:normal sampling and the resultant size will
     * depend on the overlap between the two.
     *
     * @param sample_size
     * @return sampled indices.
     */
    pcl::IndicesPtr SurfaceRegister::select_features(int sample_size) {
        int farthest_sz = sample_size*0.25;
        pcl::IndicesPtr farthest_sample = farthest_point_sample(farthest_sz);
        pcl::IndicesPtr normal_sample = normal_space_sample(sample_size-farthest_sz);
        return unique_indices(farthest_sample, normal_sample);
    }

    pcl::IndicesPtr SurfaceRegister::farthest_point_sample(int num_samples) {
        const int maxIter = 10;
        const int K = 1;

        int max_distance = 0;
        pcl::IndicesPtr max_iter_points(new std::vector<int>());

        for (int it = 0; it < maxIter; it++) {
            // random initial points
            const int sz = cloud->points.size();
            int point_indexes[num_samples];
            for (int i = 0; i < num_samples; ++i) {
                int r = std::rand() % sz + 1;
                point_indexes[i] = r;
            }

            // knn search
            pcl::KdTreeFLANN <PointT> kdtree;
            kdtree.setInputCloud(cloud);

            std::vector<int> iter_points;
            int iter_d = 0;

            for (int i = 0; i < num_samples; i++) {
                int pnt = point_indexes[i];
                PointT searchPoint = cloud->points[pnt];

                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);
                // Searching for only K when K=1 returns the same point with 0 distance.
                kdtree.nearestKSearch(searchPoint, K+1, pointIdxNKNSearch, pointNKNSquaredDistance);

                iter_points.push_back(pointIdxNKNSearch[K]);
                iter_d += pointNKNSquaredDistance[K];
            }

            if (iter_d > max_distance) {
                max_distance = iter_d;
                (*max_iter_points) = iter_points;
            }
        }

        return max_iter_points;
    }


    // https://github.com/PointCloudLibrary/pcl/blob/master/test/filters/test_sampling.cpp

    pcl::IndicesPtr SurfaceRegister::normal_space_sample(int num_samples) {
        return normal_space_sample(num_samples, 4, 50);
    }

    pcl::IndicesPtr SurfaceRegister::normal_space_sample(int num_samples, int bins, int seed) {
        pcl::NormalSpaceSampling<PointT, NormalT> sample;
        sample.setSeed(seed);
        sample.setNormals(normals);
        sample.setInputCloud(cloud);
        sample.setSample(num_samples);
        sample.setBins(bins, bins, bins);

        pcl::IndicesPtr indices(new std::vector<int>());
        sample.filter(*indices);

        return indices;
    }

    /**
     * Take the unique indicies of two samples.
     *
     * @param indices sample1 .
     * @param indices sample2 .
     * @return unique join of sample indices .
     */
    pcl::IndicesPtr SurfaceRegister::unique_indices(pcl::IndicesPtr sample1, pcl::IndicesPtr sample2) {
        pcl::IndicesPtr sample(new std::vector<int>());

        std::set<int> s;
        for (int i = 0; i < sample1->size(); i++) {
            s.insert((*sample1)[i]);
        }
        for (int i = 0; i < sample2->size(); i++) {
            s.insert((*sample2)[i]);
        }

        sample->assign(s.begin(), s.end());
        return sample;
    }

    std::vector<std::vector<float>> SurfaceRegister::compute_descriptors(pcl::IndicesPtr features) {
        return compute_descriptors(features, 10, 20);
    }

    std::vector<std::vector<float>> SurfaceRegister::compute_descriptors(pcl::IndicesPtr features, int bins, int radius) {
        std::vector<std::vector<float>> histograms;
        TOLDI_compute(cloud, *features, radius, bins, histograms);
        return histograms;
    }

}