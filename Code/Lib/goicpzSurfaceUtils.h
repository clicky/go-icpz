//
// Created by Murtuza Husain on 26/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_GOICPZSURFACEUTILS_H
#define GOICPZ_SUPERBUILD_GOICPZSURFACEUTILS_H

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/normal_space.h>
#include <pcl/filters/extract_indices.h>
#include <string>
#include <vector>
#include "TOLDI.h"



namespace goicpz {


    //PointCloudT::Ptr select_feature_points(PointCloudT::Ptr mesh, pcl::IndicesPtr feature_indices);



}


#endif //GOICPZ_SUPERBUILD_GOICPZSURFACEUTILS_H
