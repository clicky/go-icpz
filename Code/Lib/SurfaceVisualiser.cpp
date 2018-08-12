//
// Created by Murtuza Husain on 22/07/2018.
//

#include "SurfaceVisualiser.h"

namespace goicpz {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> SurfaceVisualiser::simpleVis(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2)
    {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ> (cloud1, "sample cloud1");
        viewer->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        return (viewer);
    }

}