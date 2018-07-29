//
// Created by Murtuza Husain on 18/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_GLOBALREGISTER_H
#define GOICPZ_SUPERBUILD_GLOBALREGISTER_H

#include <SurfaceRegister.h>

using namespace goicpz;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    class GlobalRegister {

    private:
        float _alpha;
        float _epsilon;
    /**
     * M = moving pre operative point cloud
     * T = target intra-operative point cloud
     */
    protected:
        MovingSurfaceRegister moving;
        TargetSurfaceRegister target;

        // Feature indexes
        pcl::IndicesPtr moving_features_idx;
        pcl::IndicesPtr target_features_idx;

        // Feature descriptors
        Eigen::MatrixXf moving_descriptors;
        Eigen::MatrixXf target_descriptors;

        // Feature distances
        std::vector<std::vector<float>> moving_feature_distances;
        std::vector<float> moving_boundary_distances;
        std::vector<std::vector<float>> target_feature_distances;
        std::vector<float> target_boundary_distances;

    public:
        GlobalRegister(float alpha, float epsilon) {
            _alpha = alpha;
            _epsilon = epsilon;
        }

        void loadMoving(std::string pathToSurface, std::string pathToMask, std::string pathToBoundary);
        void preProcessMoving();

        void loadTarget(std::string pathToSurface, std::string pathToBoundary);
        void processTarget();

        void buildCorrespondeces();

        void buildAffinityMatrix();
        float getContourConstraint(PointT mi, PointT mj, PointT ti, PointT tj);
        float applyRigidityRegulator(PointT mi, PointT mj, PointT ti, PointT tj, float sigma);
        float correspondence(PointT mi, PointT mj, PointT ti, PointT tj);
        float geodesicDistance(PointT i, PointT j);
    };

}


#endif //GOICPZ_SUPERBUILD_GLOBALREGISTER_H
