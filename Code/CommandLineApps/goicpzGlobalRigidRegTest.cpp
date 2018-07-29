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
#include <SurfaceVisualiser.h>

using namespace goicpz;



int main (int argc, char** argv) {

    GlobalRegister rigid(1, 0.0001);
    rigid.loadMoving("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialConfigurationSegmentedSurface.ply",
                     "/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialCT_top.ply",
                     "/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialCT_boundary.ply");
    rigid.preProcessMoving();

    rigid.loadTarget("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/PartialDeformed4.ply",
                     "/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/PartialDeformed4_boundary.ply");
    rigid.processTarget();

    rigid.buildCorrespondeces();
    flann::Matrix<float> W = rigid.buildAffinityMatrix(0.3);

    /*SurfaceVisualiser vis;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = vis.simpleVis(moving_featureSurface);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/

    return 0;
}
