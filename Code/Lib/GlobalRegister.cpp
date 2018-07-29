//
// Created by Murtuza Husain on 18/03/2018.
//

#include "GlobalRegister.h"
#include "math.h"
#include "nanoflann.hpp"
#include <flann/flann.hpp>

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
        int moving_rows = moving_descriptors.size();
        int moving_cols = moving_descriptors[0].size();

        int target_rows = target_descriptors.size();
        int target_cols = target_descriptors[0].size();

        flann::Matrix<float> moving_dataset(new float[moving_rows*moving_cols], moving_rows, moving_cols);
        flann::Matrix<float> target_dataset(new float[target_rows*target_cols], target_rows, target_cols);

        for (int i=0; i < moving_rows; i++) {
            for (int j=0; j<moving_cols; j++) {
                moving_dataset[i][j] = moving_descriptors[i][j];
            }
        }

        for (int i=0; i < target_rows; i++) {
            for (int j=0; j<target_cols; j++) {
                target_dataset[i][j] = target_descriptors[i][j];
            }
        }

        int nn = 2;

        flann::Matrix<int> indices(new int[target_dataset.rows*nn], target_dataset.rows, nn);
        flann::Matrix<float> dists(new float[target_dataset.rows*nn], target_dataset.rows, nn);
        // construct an randomized kd-tree index using 4 kd-trees
        flann::Index<flann::L2<float> > index(moving_dataset, flann::KDTreeIndexParams(4));
        index.buildIndex();
        // do a knn search, using 128 checks
        index.knnSearch(target_dataset, indices, dists, nn, flann::SearchParams(128));

        // 1st col: idx features in target mdoel
        // 2nd col: corresponding idx in moving model
        // 3rd col: distance between the corresponding descriptors
        //int rows = indices.rows*nn;
        //flann::Matrix<float> candidates(new int float[rows*3], rows, 3);

        ic_indexes = indices;
        ic_distances = dists;
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
    flann::Matrix<float> GlobalRegister::buildAffinityMatrix(float sigma) {
        int rows = ic_indexes.rows;
        int cols = ic_indexes.rows;

        flann::Matrix<float> W(new float[rows*cols], rows, cols);

        for (int i=0; i<rows; i++) {
            int col = i >= rows ? 1 : 0;
            for (int j=0; j<cols; j++) {
                if (i == j) {
                    W[i][j] = ic_distances[i][col];
                } else {
                    int idx_mi = ic_indexes[i][0];
                    int idx_mj = ic_indexes[j][0];

                    int idx_ti = i;
                    int idx_tj = j;

                    float dm = moving_feature_distances[idx_mi][idx_mj];
                    float dbmi = moving_boundary_distances[idx_mi];
                    float dbmj = moving_boundary_distances[idx_mj];
                    
                    float dt = target_feature_distances[idx_ti][idx_tj];
                    float dbti = target_boundary_distances[idx_ti];
                    float dbtj = target_boundary_distances[idx_tj];

                    W[i][j] = _alpha * applyRigidityRegulator(dm, dt, sigma) +
                              (1 - _alpha) * getContourConstrafloat(dbmi, dbmj, dbti, dbtj, sigma);
                }
            }
        }

        return W;
    }

    /**
     * g_b
     *
     * g_b(m_i,m_j,t_i,t_j,sigma_x) = 0.5 * (g(m_i,x_mi,t_i,x_ti,sigma_x) + g(m_i,x_mj,t_i,x_tj,sigma_x))
     */
    float GlobalRegister::getContourConstrafloat(float mb_i, float mb_j, float tb_i, float tb_j, float sigma) {
        return 0.5 * (applyRigidityRegulator(mb_i, tb_i, sigma) + applyRigidityRegulator(mb_j, tb_j, sigma));
    }

    /**
     * g
     *
     * g(m_i,m_j,t_i,t_j,sigma) = exp( (c(m_i,m_j,t_i,t_j) - 1)^2 / (2 * sigma^2) )
     */
    float GlobalRegister::applyRigidityRegulator(float m_ij, float t_ij, float sigma) {
        return exp(pow((correspondence(m_ij, t_ij) - 1), 2) / (2 * pow(sigma, 2)));
    }

    /**
     * c
     *
     * c(m_i,m_j,t_i,t_j) = min [ (d(m_i,m_j) / (d(t_i,t_j) + epsi), (d(t_i,t_j) / (d(m_i,m_j) + epsi) ]
     */
    float GlobalRegister::correspondence(float m_ij, float t_ij) {
        float a = m_ij / (t_ij + _epsilon);
        float b = t_ij / (m_ij + _epsilon);
        return std::min(a, b);
    }
}