//
// Created by Murtuza Husain on 26/03/2018.
//

#include "goicpzSurfaceUtils.h"

namespace goicpz {


    /**
     * Extract a subset of points from a point cloud.
     *
     * @param mesh to subsample.
     * @param feature_indices of input mesh required.
     * @return subset of input mesh.
     *
     * @see http://pointclouds.org/documentation/tutorials/extract_indices.php
     */
    /*PointCloudT::Ptr select_feature_points(PointCloudT::Ptr mesh, pcl::IndicesPtr feature_indices) {
        PointCloudT::Ptr features(new PointCloudT);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(mesh);
        extract.setIndices(feature_indices);
        extract.setNegative(false);
        extract.filter(*features);

        return features;
    }*/


}