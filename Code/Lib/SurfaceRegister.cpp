//
// Created by Murtuza Husain on 21/07/2018.
//

#include <pcl/features/normal_3d.h>
#include "SurfaceRegister.h"
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/geometry.h>

namespace goicpz {

    /**
     * Load the
     * @param path
     */
    void SurfaceRegister::load_surface(std::string surfacePath, std::string boundaryPath) {
        read_ply(surfacePath, surface);
        read_ply(boundaryPath, boundary);
    }

    void MovingSurfaceRegister::load_surface(std::string surfacePath, std::string maskPath, std::string boundaryPath) {
        SurfaceRegister::load_surface(surfacePath, boundaryPath);
        read_ply(maskPath, mask);
    }

    void SurfaceRegister::read_ply(std::string path, PointCloudT::Ptr cloud) {
        if (pcl::io::loadPLYFile(path, *cloud) < 0) {
            std::cerr << "Error loading cloud " << path << std::endl;
        } else {
            std::cout << "Cloud loaded from " << path << std::endl;
        }
    }

    void SurfaceRegister::compute_surface_normals() {
        pcl::NormalEstimation<PointT,NormalT> ne;
        ne.setInputCloud (surface);
        ne.setRadiusSearch (0.05);
        ne.compute(*normals);
        copyPointCloud(*surface, *normals);
    }

    // FEATURE SELECTION

    /**
     * Sample the point cloud surface using a combination of farthest point and normal space sampling, returning the
     * unique intersection of the two. A 1:3 split is used for farthest:normal sampling and the resultant size will
     * depend on the overlap between the two.
     *
     * @param sample_size
     * @return sampled indices.
     */
    pcl::IndicesPtr SurfaceRegister::sample_features(int sample_size) {
        int farthest_sz = sample_size*0.25;

        pcl::IndicesPtr farthest_sample = farthest_point_sample(farthest_sz);
        pcl::IndicesPtr normal_sample = normal_space_sample(sample_size-farthest_sz);
        pcl::IndicesPtr sample = unique_indices(farthest_sample, normal_sample);

        return remove_boundary_points(surface, sample);
    }

    pcl::IndicesPtr SurfaceRegister::farthest_point_sample(int num_samples) {
        const int maxIter = 10;
        const int K = 1;

        int max_distance = 0;
        pcl::IndicesPtr max_iter_points(new std::vector<int>());

        for (int it = 0; it < maxIter; it++) {
            // random initial points
            const int sz = surface->points.size();
            int point_indexes[num_samples];
            for (int i = 0; i < num_samples; ++i) {
                int r = std::rand() % sz + 1;
                point_indexes[i] = r;
            }

            // knn search
            pcl::KdTreeFLANN <PointT> kdtree;
            kdtree.setInputCloud(surface);

            std::vector<int> iter_points;
            int iter_d = 0;

            for (int i = 0; i < num_samples; i++) {
                int pnt = point_indexes[i];
                PointT searchPoint = surface->points[pnt];

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

    /**
     * S
     * @param num_samples
     * @return
     * @see https://github.com/PointCloudLibrary/pcl/blob/master/test/filters/test_sampling.cpp
     */
    pcl::IndicesPtr SurfaceRegister::normal_space_sample(int num_samples) {
        return normal_space_sample(num_samples, 4, 50);
    }

    /**
     *
     * @param num_samples
     * @param bins
     * @param seed
     * @return
     */
    pcl::IndicesPtr SurfaceRegister::normal_space_sample(int num_samples, int bins, int seed) {
        pcl::NormalSpaceSampling<PointT, NormalT> sample;
        sample.setSeed(seed);
        sample.setNormals(normals);
        sample.setInputCloud(surface);
        sample.setSample(num_samples);
        sample.setBins(bins, bins, bins);

        pcl::IndicesPtr indices(new std::vector<int>());
        sample.filter(*indices);

        return indices;
    }

    pcl::IndicesPtr SurfaceRegister::remove_boundary_points(PointCloudT::Ptr surface, pcl::IndicesPtr points) {
        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(boundary);

        const int K = 1;
        pcl::IndicesPtr fixed(new std::vector<int>());

        for (int i=0; i<points->size(); i++) {
            int idx = (*points)[i];
            PointT point = surface->points[idx];

            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            kdtree.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance);

            if (pointNKNSquaredDistance[0] > 0) {
                fixed->push_back(idx);
            }
        }

        return fixed;
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

    pcl::IndicesPtr SurfaceRegister::select_feature_points(PointCloudT::Ptr surface, int sample_size) {
        // Select feature points
        pcl::IndicesPtr idxFeatures = sample_features(sample_size);

        // Get feature points from mask surface using nn search
        const int K = 1;

        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(surface);

        feature_indexes->clear();

        for (int idxFeature : (*idxFeatures)) {
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            PointT point = SurfaceRegister::surface->points[idxFeature];
            kdtree.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
            feature_indexes->push_back(pointIdxNKNSearch[0]);
        }

        return feature_indexes;
    }

    PointCloudT::Ptr SurfaceRegister::extract_points(PointCloudT::Ptr mesh, pcl::IndicesPtr feature_indices) {
        PointCloudT::Ptr features(new PointCloudT);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(mesh);
        extract.setIndices(feature_indices);
        extract.setNegative(false);
        extract.filter(*features);

        return features;
    }

    // DESCRIPTORS

    Eigen::MatrixXf SurfaceRegister::compute_descriptors(PointCloudT::Ptr surface, pcl::IndicesPtr features) {
        return compute_descriptors(surface, features, 20, 10);
    }

    Eigen::MatrixXf SurfaceRegister::compute_descriptors(PointCloudT::Ptr surface, pcl::IndicesPtr features, int bins, int radius) {
        std::vector<std::vector<float>> descriptorsToldi;
        TOLDI_compute(surface, *features, radius, bins, descriptorsToldi);

        int numFeatures = features->size();
        Eigen::MatrixXf descriptorsToldiOut(numFeatures, descriptorsToldi[0].size());
        for (int i = 0; i < numFeatures; i++) {
            descriptorsToldiOut.row(i) = Eigen::VectorXf::Map(&descriptorsToldi[i][0], descriptorsToldi[i].size());
        }

        return descriptorsToldiOut;
    }

    // DISTANCES

    /**
     * Compute matrix of squared Euclidean distances between surface points for features.
     *
     * @param surface to compute distances for.
     * @return distance matrix.
     */
    std::vector<std::vector<float>> SurfaceRegister::compute_distances(PointCloudT::Ptr surface) {
        distances_features.clear();
        int sz = surface->points.size();
        for (int i=0; i<sz; i++) {
            distances_features.push_back(std::vector<float>());
            for (int j=0; j<sz; j++) {
                distances_features[i].push_back(i == j ? 0 :
                                                pcl::geometry::squaredDistance(surface->points[i], surface->points[j]));
            }
        }
        return distances_features;
    }

    /**
     * Compute the distance between features on a surface and the boundary.
     *
     * @param surface to take features from
     * @param feature point indexes on provided surface
     * @return distance vector to nearest point on boundary for each surface feature
     */
    std::vector<float> SurfaceRegister::compute_boundary_distances(PointCloudT::Ptr surface, pcl::IndicesPtr features) {
        distances_feature_boundary.clear();

        const int K = 1;

        pcl::KdTreeFLANN <PointT> kdtree;
        kdtree.setInputCloud(boundary);

        for (int idxFeature : (*features)) {
            PointT point = surface->points[idxFeature];

            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            kdtree.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
            distances_feature_boundary.push_back(pointNKNSquaredDistance[0]);
        }

        return distances_feature_boundary;
    }

    // INTERPOLATION

    PointNormalCloudT TargetSurfaceRegister::interpolate_surface() {
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        PointNormalCloudT mls_points;

        pcl::MovingLeastSquares<PointT, NormalT> mls;

        mls.setComputeNormals (true);

        // Set parameters
        mls.setInputCloud(surface);
        //mls.setPolynomialOrder(2);
        mls.setPolynomialFit (true);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.03);

        // Reconstruct
        mls.process(mls_points);

        return mls_points;
    }

    // UTILS

    void SurfaceRegister::save() {

    }

    PointCloudT::Ptr SurfaceRegister::getSurface() {
        return surface;
    }

    PointCloudT::Ptr MovingSurfaceRegister::getMask() {
        return mask;
    }

    PointCloudT::Ptr SurfaceRegister::getSurfaceBoundary() {
        return boundary;
    }

    PointNormalCloudT::Ptr SurfaceRegister::getNormals() {
        return normals;
    }

    pcl::IndicesPtr SurfaceRegister::getFeatureIndexes() {
        return feature_indexes;
    }

}