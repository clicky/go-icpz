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
        pcl::IndicesPtr features_idx;
        std::vector<std::vector<float>> histograms;
    public:
        void load_moving_surface(std::String path);
        void load_moving_top(std::String path);
        void load_moving_boundary(std::String path);
        PointCloudT::Ptr build_features();
        void build_descriptors();
    };

}


#endif //GOICPZ_SUPERBUILD_ICPPREPROCESSOR_H
