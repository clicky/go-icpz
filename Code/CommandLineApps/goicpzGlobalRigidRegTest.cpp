//
// Created by Murtuza Husain on 26/03/2018.
//

#include <iostream>
#include <goicpzSurfaceUtils.h>
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

    // Pre-process moving (pre operative) surface
    MovingSurfaceRegister reg;

    // Read in point clouds
    reg.read_ply("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialConfigurationSegmentedSurface.ply", reg.getSurface());
    reg.read_ply("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialCT_top.ply", reg.getMask());
    reg.read_ply("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/InitialCT_boundary.ply", reg.getSurfaceBoundary());

    // Estimate surface normals
    reg.compute_surface_normals();

    // Select features
    pcl::IndicesPtr features = reg.select_feature_points(1500);

    // Get feature point descriptors (TOLDI)
    std::vector<std::vector<float>> moving_descriptors = reg.compute_descriptors(reg.getMask(), features);

    // Distances
    PointCloudT::Ptr featureSurface = reg.extract_points(reg.getMask(), features);
    std::vector<std::vector<float>> distances_features_moving = reg.compute_distances(featureSurface);
    std::vector<float> distances_boundary_moving = reg.compute_boundary_distances(reg.getMask(), reg.getFeatureIndexes());

    TargetSurfaceRegister target;
    target.read_ply("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/PartialDeformed4.ply", target.getSurface());
    target.read_ply("/Users/murtuza/dev/Matlab/data/IPCAI2018Data/reform/PartialDeformed4_boundary.ply", target.getSurfaceBoundary());

    PointNormalCloudT interp = target.interpolate_surface();

    SurfaceVisualiser vis;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = vis.simpleVis(featureSurface);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
