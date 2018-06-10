//
// Created by Murtuza Husain on 26/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_ICPPREPROCESSOR_H
#define GOICPZ_SUPERBUILD_ICPPREPROCESSOR_H


typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    class IcpPreProcessor {
    private:
        PointCloudT::Ptr movingMesh_surface;
        PointCloudT::Ptr movingMesh_top;
        PointCloudT::Ptr movingMesh_boundary;
    public:
        void loadMesh(std::String path, PointCloudT::Ptr mesh);
        PointCloudT::Ptr build_features();
    };

}


#endif //GOICPZ_SUPERBUILD_ICPPREPROCESSOR_H
