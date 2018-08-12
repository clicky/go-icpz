//
// Created by Murtuza Husain on 18/03/2018.
//

#include "GlobalRegister.h"
#include "math.h"
#include "nanoflann.hpp"
#include <flann/flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <omp.h>

namespace goicpz {

    /**
     * Load PLY files into point clouds for the moving surface.
     *
     * @param pathToSurface
     * @param pathToMask
     * @param pathToBoundary
     */
    void GlobalRegister::loadMoving(std::string pathToSurface, std::string pathToMask, std::string pathToBoundary) {
        moving.read_ply(pathToSurface, moving.getSurface());
        moving.read_ply(pathToMask, moving.getMask());
        moving.read_ply(pathToBoundary, moving.getSurfaceBoundary());
    }

    /**
     * Pre process moving surface by selecting features, computing their descriptors and the pairwise distance
     * constraints. Uses a combination of normal space and farthest point sampling for feature selection.
     *
     * @param sampleSize
     */
    void GlobalRegister::preProcessMoving(int sampleSize) {
        // Estimate surface normals
        moving.compute_surface_normals();

        // Select features
        moving_features_idx = moving.select_feature_points(moving.getSurface(), sampleSize);

        // Get feature point descriptors (TOLDI)
        moving_descriptors = moving.compute_descriptors(moving.getSurface(), moving_features_idx);

        // Distances
        PointCloudT::Ptr moving_featureSurface = moving.extract_points(moving.getSurface(), moving_features_idx);
        moving_feature_distances = moving.compute_distances(moving_featureSurface);
        moving_boundary_distances = moving.compute_boundary_distances(moving.getSurface(), moving.getFeatureIndexes());
    }

    /**
     * Load the target surface from PLY files.
     */
    void GlobalRegister::loadTarget(std::string pathToSurface, std::string pathToBoundary) {
        target.read_ply(pathToSurface, target.getSurface());
        target.read_ply(pathToBoundary, target.getSurfaceBoundary());
    }

    /**
     * Process the target surface by selecting features, building their descriptors and pairwise distances.
     *
     * @param sampleSize
     */
    void GlobalRegister::processTarget(int sampleSize) {
        target.compute_surface_normals();

        target_features_idx = target.select_feature_points(target.getSurface(), sampleSize);
        target_descriptors = target.compute_descriptors(target.getSurface(), target_features_idx);

        PointCloudT::Ptr target_featureSurface = target.extract_points(target.getSurface(), target_features_idx);
        target_feature_distances = target.compute_distances(target_featureSurface);
        target_boundary_distances = target.compute_boundary_distances(target.getSurface(), target.getFeatureIndexes());
    }

    /**
     * Build the initial correspondence from features selected from the moving and target surfaces.
     */
    void GlobalRegister::buildCorrespondeces() {
        int moving_rows = moving_descriptors.size();
        int moving_cols = moving_descriptors[0].size();

        int target_rows = target_descriptors.size();
        int target_cols = target_descriptors[0].size();

        flann::Matrix<float> moving_dataset(new float[moving_rows*moving_cols], moving_rows, moving_cols);
        flann::Matrix<float> target_dataset(new float[target_rows*target_cols], target_rows, target_cols);

        #pragma omp parallel num_threads(2) {
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
        }

        int nn = 2;

        flann::Matrix<int> indices(new int[target_dataset.rows*nn], target_dataset.rows, nn);
        flann::Matrix<float> dists(new float[target_dataset.rows*nn], target_dataset.rows, nn);
        // construct an randomized kd-tree index using 4 kd-trees
        flann::Index<flann::L2<float> > index(moving_dataset, flann::KDTreeIndexParams(4));
        index.buildIndex();
        // do a knn search, using 128 checks
        index.knnSearch(target_dataset, indices, dists, nn, flann::SearchParams(128));

        ic_indexes = indices;
        ic_distances = dists;
    }

    /**
     * Compute the affinity matrix W for the intial correspondence
     *
     * i == j
     * W_ij = sim(f(m_i),f(t_i)
     *
     * i != j
     * W_ij = alpha * g_d(m_i,m_j,t_i,t_j,sigma_d) + (1-alpha) * g_b(m_i,m_j,t_i,t_j,sigma_b)
     *
     * @param sigma
     */
    Eigen::MatrixXf GlobalRegister::buildAffinityMatrix(float sigma) {
        int rows = ic_indexes.rows;
        int cols = ic_indexes.rows;

        Eigen::MatrixXf W(rows, cols);

        for (int i=0; i<rows; i++) {
            int col = i >= rows ? 1 : 0;
            for (int j=0; j<cols; j++) {
                if (i == j) {
                    W(i, j) = ic_distances[i][col];
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

                    W(i, j) = _alpha * applyRigidityRegulator(dm, dt, sigma) +
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

    bool asc_comp(float i, float j) {
        return i < j;
    }

    bool desc_comp(float i, float j) {
        return i > j;
    }

    /**
     * Vector sorting with ID's of the original elements in sorted order.
     *
     * @param unsorted original array to be sorted
     * @param sorted array to store result
     * @param idx id's original array in order of result
     * @param side 1 for ascending sort or 2 for descending sort
     */
    void sort(Eigen::VectorXf unsorted, std::vector<float> &sorted, std::vector<int> &idx, int side) {
        std::map<float, int> mp;
        const int sz = unsorted.size();
        for (int i=0; i<sz; i++) {
            float val = unsorted(i);
            sorted.push_back(val);
            mp[val] = i;
        }
        std::sort(sorted.begin(), sorted.end(), side == 1 ? asc_comp : desc_comp);
        for (int i=0; i<sz; i++) {
            idx.push_back(mp[sorted[i]]);
        }
    }

    void sort_asc(Eigen::VectorXf unsorted, std::vector<float> &sorted, std::vector<int> &idx) {
        sort(unsorted, sorted, idx, 1);
    }

    void sort_desc(Eigen::VectorXf unsorted, std::vector<float> &sorted, std::vector<int> &idx) {
        sort(unsorted, sorted, idx, 2);
    }

    /**
     * Prune the initial correspondence by performing spectral analysis on the affinity matrix W.
     * The final correspondence for both the moving and target surfaces is stored in the provided vectors.
     *
     * @param W
     * @param rigidity_threshold
     * @param moving_correspondence vector pointer for correspondence of points on moving surface.
     * @param target_correspondence vector pointer for correspondence of points on target surface.
     */
    void GlobalRegister::prune_correspondence(
            Eigen::MatrixXf W, float rigidity_threshold, pcl::IndicesPtr &moving_correspondence,
            pcl::IndicesPtr &target_correspondence
    ) {
        #pragma omp parallel {
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        };
        std::cout << "SVD computed" << std::endl;

        Eigen::MatrixXf U = svd.matrixU();
        Eigen::VectorXf lead_eig = U.col(0);

        const int sz = lead_eig.size();
        bool final [sz];

        std::vector<float> sorted;
        std::vector<int> idx;
        sort_desc(lead_eig, sorted, idx);

        int final_sz = 0;
        std::vector<float> confidence_matches;

        for (int i=0; i<sz; i++) {
            if (final[i] == false) {
                Eigen::VectorXf v = W.col(i);
                for (int j=0; j<v.size(); j++) {
                    if (v[j] > rigidity_threshold) {
                        confidence_matches.push_back(lead_eig[j]);
                        idx.push_back(j);
                        int k = ic_indexes[j][0];
                        moving_correspondence->push_back((*moving_features_idx)[k]);
                        target_correspondence->push_back((*target_features_idx)[k]);
                        final_sz++;
                    } else {
                        final[j] = true;
                    }
                }
            }
        }
    }

    /**
     * Estimate the rigid transformation between the final correspondences and return the transformed moving surface.
     *
     * @param moving_correspondence
     * @param target_correspondence
     * @return transformed moving surface.
     */
    PointCloudT::Ptr GlobalRegister::transform(pcl::IndicesPtr moving_correspondence, pcl::IndicesPtr target_correspondence) {
        PointCloudT::Ptr mc (new PointCloudT);
        mc->width = ic_indexes.rows;
        mc->height = 1;
        mc->is_dense = false;
        mc->resize(mc->width * mc->height);

        PointCloudT::Ptr tc (new PointCloudT);
        tc->width = ic_indexes.rows;
        tc->height = 1;
        tc->is_dense = false;
        tc->resize(tc->width * tc->height);

        for (int i=0; i<moving_correspondence->size(); i++) {
            int idx = (*target_correspondence)[i];
            if (idx < ic_indexes.rows) {
                mc->points[i] = moving.getSurface()->points[idx];
                tc->points[i] = target.getSurface()->points[idx];
            }
        }

        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation2;
        TESVD.estimateRigidTransformation (*mc,*tc,transformation2);

        //PointCloudT::Ptr m = moving.extract_points(moving.getMask(), ic_indexes);
        //PointCloudT::Ptr t = target.extract_points(target.getSurface(), ic_indexes);

        printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2));
        printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2));
        printf ("\n");
        printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));

        Eigen::Matrix3d r;
        r(0,0) = transformation2(0,0);
        r(0,0) = transformation2(0,1);
        r(0,0) = transformation2(0,2);
        r(0,0) = transformation2(1,0);
        r(0,0) = transformation2(1,1);
        r(0,0) = transformation2(1,2);
        r(0,0) = transformation2(2,0);
        r(0,0) = transformation2(2,1);
        r(0,0) = transformation2(2,2);

        PointCloudT::Ptr tt (new PointCloudT);
        tt->width = ic_indexes.rows;
        tt->height = 1;
        tt->is_dense = false;
        tt->resize(tt->width * tt->height);
        for (int i=0; i<ic_indexes.rows; i++) {
            PointT p = mc->points[i];

            Eigen::Vector3d vp;
            vp(0) = p.x;
            vp(1) = p.y;
            vp(2) = p.z;

            Eigen::Vector3d res = r * vp;

            tt->points[i].x = res(0) + transformation2(0,3);
            tt->points[i].y = res(1) + transformation2(1,3);
            tt->points[i].z = res(2) + transformation2(2,3);

        }

        return tt;
    }
}