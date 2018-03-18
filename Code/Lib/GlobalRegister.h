//
// Created by Murtuza Husain on 18/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_GLOBALREGISTER_H
#define GOICPZ_SUPERBUILD_GLOBALREGISTER_H


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    class GlobalRegister {
    protected:
        PointCloudT::Ptr cloud_moving;
        PointCloudT::Ptr cloud_target;
    /**
     * M = moving pre operative point cloud
     * T = target intra-operative point cloud
     */
    public:
        GlobalRegister():
                cloud_moving(new PointCloudT),
                cloud_target(new PointCloudT)
        {}
        void buildCorrespondeces();
    };

    class RigidRegister: GlobalRegister {
    private:
        pcl::KDTree graph_moving;
        pcl::KDTree graph_target;
        float _alpha;
        float _epsilon;
    public:
        RigidRegister(float alpha, float epsilon) {
            _alpha = alpha;
            _epsilon = epsilon;
        }
        void buildAffinityMatrix();
        float getContourConstraint(PointT mi, PointT mj, PointT ti, PointT tj);
        float applyRigidityRegulator(PointT mi, PointT mj, PointT ti, PointT tj);
        float correspondence(PointT mi, PointT mj, PointT ti, PointT tj);
        float geodesicDistance(PointT i, PointT j);
    };

}


#endif //GOICPZ_SUPERBUILD_GLOBALREGISTER_H
