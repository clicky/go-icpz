//
// Created by Murtuza Husain on 26/03/2018.
//

#include <iostream>
#include <goicpzSurfaceUtils.h>
#include <GlobalRegister.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <SurfaceRegister.h>
#include <boost/timer.hpp>

using namespace goicpz;



int main (int argc, char** argv) {
    boost::timer timer;
    int sampleSize = 600;

    GlobalRegister rigid(1, 0.0001);
    rigid.loadMoving("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialCT_top.ply",
                     "/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialCT_top.ply",
                     "/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialCT_boundary.ply");
    rigid.preProcessMoving(sampleSize);
    std::cout << timer.elapsed() << std::endl;
    timer.restart();

    rigid.loadTarget("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/PartialDeformed4.ply",
                     "/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/PartialDeformed4_boundary.ply");
    rigid.processTarget(sampleSize);
    std::cout <<  timer.elapsed() << std::endl;
    timer.restart();

    rigid.buildCorrespondeces();
    std::cout <<  timer.elapsed() << std::endl;
    timer.restart();

    Eigen::MatrixXf W = rigid.buildAffinityMatrix(0.3);
    std::cout <<  timer.elapsed() << std::endl;
    timer.restart();

    pcl::IndicesPtr moving_correspondence(new std::vector<int>);
    pcl::IndicesPtr target_correspondence(new std::vector<int>);

    rigid.prune_correspondence(W, 0.4, moving_correspondence, target_correspondence);
    std::cout << "Prune " <<  timer.elapsed() << std::endl;
    timer.restart();

    std::cout << rigid.moving_features_idx->size() << std::endl;

    int x = 9;

    /*PointCloudT::Ptr res = rigid.transform(moving_correspondence, target_correspondence);
    std::cout << "Transform " <<  timer.elapsed() << std::endl;
    timer.restart();*/



    return 0;
}
