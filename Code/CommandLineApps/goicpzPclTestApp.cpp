/*=============================================================================

  GOICPZ: A software package for globally optimal implementations of the iterative closest point algorithm.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <goicpzMyFunctions.h>
#include <iostream>

#ifdef BUILD_gflags
#include "gflags/gflags.h"
#endif

#ifdef BUILD_glog
#include <glog/logging.h>
#endif

#ifdef BUILD_Eigen
#include <Eigen/Dense>
#endif

#ifdef BUILD_Boost
#include <boost/math/special_functions/round.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/path.hpp>
#endif

#ifdef BUILD_OpenCV
#include <cv.h>
#endif

#ifdef BUILD_PCL
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#endif

#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <goicpzPclIcp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char** argv)
{
    goicpz::PclRegister pcl;
    pcl.registerFixedSurface(argv[1]);
    pcl.registerTargetSurface(argv[2]);
    pcl.performIcp();

    return 0;
}
