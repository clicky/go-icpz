//
// Created by Murtuza Husain on 26/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_ICPPREPROCESSOR_H
#define GOICPZ_SUPERBUILD_ICPPREPROCESSOR_H


typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

    class IcpPreProcessor {
    private:
        PointCloudT::Ptr movingMesh;
    public:
        void loadMovingMesh(std::String path);
        PointCloudT::Ptr selectFeatures();
        void computeDescriptors(PointCloudT::Ptr mask);
        void buildDistanceTree();
    };

}


#endif //GOICPZ_SUPERBUILD_ICPPREPROCESSOR_H
