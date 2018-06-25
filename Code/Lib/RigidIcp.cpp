#include "RigidIcp.h"
#include <pcl/surface/poisson.h>

namespace goicpz {

  void RigidIcp::load_mesh(std::String) {
    if (pcl::io::loadPLYFile(path, *target) < 0) {
        std::cerr << "Error loading cloud " << path << std::endl;
    }
    std::cout << "Cloud loaded from " << path << std::endl;

    // Surface reconstruction
    // https://github.com/PointCloudLibrary/pcl/blob/master/tools/poisson_reconstruction.cpp

    int default_depth = 8;
    int default_solver_divide = 8;
    int default_iso_divide = 8;
    float default_point_weight = 4.0f;

    pcl::Poisson<PointNormal> poisson;
  	poisson.setDepth (default_depth);
  	poisson.setSolverDivide (default_solver_divide);
  	poisson.setIsoDivide (default_iso_divide);
    poisson.setPointWeight (default_point_weight);
    poisson.setInputCloud (target);
    poisson.reconstruct (reconstructed);

    //TODO loadIntraopData.m > 47, resoloution adjustment?


  }

  pcl::IndicesPtr RigidIcp::select_features(int num_features) {
    MeshSampler sampler;
    pcl::IndicesPtr sample = sampler.fartest_sample(target, num_features);

    //TODO remove bdry points

    return sample;
  }

}
