//
// Created by Murtuza Husain on 26/03/2018.
//

#include "IcpPreProcessor.h"
#include <math.h>

namespace goicpz {

    void IcpPreProcessor::loadMovingMesh(std::String path) {
        if (pcl::io::loadPLYFile(path, *movingMesh) < 0) {
            std::cerr << "Error loading cloud " << path << std::endl;
        }
        std::cout << "Cloud loaded from " << path << std::endl;
    }

    PointCloudT::Ptr IcpPreProcessor::selectFeatures() {
        int num_bins = 4;
        int num_samples = 100;

        // Estimate the surface normals
        // Estimate the normals of the cloud_xyz
        pcl::PointCloud<NormalT >::Ptr cloud_normals (new pcl::PointCloud<NormalT>);
        pcl::search::KdTree<PointT>::Ptr tree_n(new pcl::search::KdTree<PointT>());

        movingMesh->setInputCloud(cloud_xyz);
        movingMesh->setSearchMethod(tree_n);
        movingMesh->setRadiusSearch(0.2);
        movingMesh->compute(*cloud_normals);

        // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
        for(size_t i = 0; i < cloud_normals->points.size(); ++i) {
            cloud_normals->points[i].x = movingMesh->points[i].x;
            cloud_normals->points[i].y = movingMesh->points[i].y;
            cloud_normals->points[i].z = movingMesh->points[i].z;
        }

        // Get the dimensional bin sizes
        vector<double> bin_edges = linspace(-1, 1, num_bins);

        pcl::NormalSpaceSampling<<PointT, PointT>> sample;
        sample.setSeed(50);
        sample.set(*cloud_normals);
        sample.setSample(num_samples);
        sample.setBins(num_bins, num_bins, num_bins);
    }

    vector<double> linspace(int min, int max, int count) {
        vector<double> result;
        double interval = (max - min) / count;
        for (double i = min; i <= count; i++) {
            result.push_back(min + i * interval);
        }
        return result;
    }

}