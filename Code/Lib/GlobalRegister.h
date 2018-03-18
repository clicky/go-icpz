//
// Created by Murtuza Husain on 18/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_GLOBALREGISTER_H
#define GOICPZ_SUPERBUILD_GLOBALREGISTER_H


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    class GlobalRegister {

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
    public:
        void buildAffinityMatrix();
        void getContourConstraint();
        void applyRigidityRegulator();
        void correspondence();
        void geodesicDistance(PointT i, PointT j);
    };

}


#endif //GOICPZ_SUPERBUILD_GLOBALREGISTER_H
