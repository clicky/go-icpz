//
// Created by Murtuza Husain on 22/07/2018.
//

#ifndef GOICPZ_SURFACEVISUALISER_H
#define GOICPZ_SURFACEVISUALISER_H

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "SurfaceRegister.h"

namespace goicpz {

    class SurfaceVisualiser {
    public:
        boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2);

    };

}


#endif //GOICPZ_SURFACEVISUALISER_H
