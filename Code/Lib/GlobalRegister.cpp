//
// Created by Murtuza Husain on 18/03/2018.
//

#include "GlobalRegister.h"
#include "math.h"

namespace goicpz {


    void GlobalRegister::loadMoving(std::string pathToSurface, std::string pathToMask, std::string pathToBoundary) {
        moving.read_ply(pathToSurface, moving.getSurface());
        moving.read_ply(pathToMask, moving.getMask());
        moving.read_ply(pathToBoundary, moving.getSurfaceBoundary());
    }

    void GlobalRegister::preProcessMoving() {
        // Estimate surface normals
        moving.compute_surface_normals();

        // Select features
        moving_features_idx = moving.select_feature_points(moving.getMask(), 1500);

        // Get feature point descriptors (TOLDI)
        moving_descriptors = moving.compute_descriptors(moving.getMask(), moving_features_idx);

        // Distances
        PointCloudT::Ptr moving_featureSurface = moving.extract_points(moving.getMask(), moving_features_idx);
        moving_feature_distances = moving.compute_distances(moving_featureSurface);
        moving_boundary_distances = moving.compute_boundary_distances(moving.getMask(), moving.getFeatureIndexes());
    }

    void GlobalRegister::loadTarget(std::string pathToSurface, std::string pathToBoundary) {
        target.read_ply(pathToSurface, target.getSurface());
        target.read_ply(pathToBoundary, target.getSurfaceBoundary());
    }

    void GlobalRegister::processTarget() {
        target.compute_surface_normals();

        target_features_idx = target.select_feature_points(target.getSurface(), 1500);
        target_descriptors = target.compute_descriptors(target.getSurface(), target_features_idx);

        PointCloudT::Ptr target_featureSurface = target.extract_points(target.getSurface(), target_features_idx);
        target_feature_distances = target.compute_distances(target_featureSurface);
        target_boundary_distances = target.compute_boundary_distances(target.getSurface(), target.getFeatureIndexes());
    }

    void GlobalRegister::buildCorrespondeces() {

    }

    /**
     * W
     *
     * i == j
     * W_ij = sim(f(m_i),f(t_i)
     *
     * i != j
     * W_ij = alpha * g_d(m_i,m_j,t_i,t_j,sigma_d) + (1-alpha) * g_b(m_i,m_j,t_i,t_j,sigma_b)
     */
    void GlobalRegister::buildAffinityMatrix() {

    }

    /**
     * g_x
     *
     * g_x(m_i,m_j,t_i,t_j,sigma_x) = 0.5 * (g(m_i,x_mi,t_i,x_ti,sigma_x) + g(m_i,x_mj,t_i,x_tj,sigma_x))
     */
    float GlobalRegister::getContourConstraint(PointT mi, PointT mj, PointT ti, PointT tj) {
        //applyRigidityRegulator(mi, j) + applyRigidityRegulator()
    }

    /**
     * g
     *
     * g(m_i,m_j,t_i,t_j,sigma) = exp( (c(m_i,m_j,t_i,t_j) - 1)^2 / (2 * sigma^2) )
     */
    float GlobalRegister::applyRigidityRegulator(PointT mi, PointT mj, PointT ti, PointT tj, float sigma) {
        return pow((correspondence(mi, mj, ti, tj) - 1), 2) / (2 * pow(sigma, 2));
    }

    /**
     * c
     *
     * c(m_i,m_j,t_i,t_j) = min [ (d(m_i,m_j) / (d(t_i,t_j) + epsi), (d(t_i,t_j) / (d(m_i,m_j) + epsi) ]
     */
    float GlobalRegister::correspondence(PointT mi, PointT mj, PointT ti, PointT tj) {
        float a = geodesicDistance(mi, mj) / geodesicDistance(ti, tj) + _epsilon;
        float b = geodesicDistance(ti, tj) / geodesicDistance(mi, mj) + _epsilon;
        return std::min(a, b);
    }

    /**
     * d
     *
     * d(x_i,x_j)
     */
    float GlobalRegister::geodesicDistance(PointT i, PointT j) {

    }
}