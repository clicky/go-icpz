#include "Sampling.h"

namespace goicpz {

  pcl::IndicesPtr MeshSampler::farthest_points(PointCloudT::Ptr mesh, int num_samples) {
      const int maxIter = 10;

      // random initial points
      const int sz = mesh->points.size();
      int point_indexes[num_samples];
      for (int i = 0; i < n; ++i) {
          point_indexes[i] = std::rand() % sz + 1;
      }

      // knn search
      pcl::KdTreeFLANN <PointT> kdtree;
      kdtree.setInputCloud(mesh);

      pcl::IndicesPtr fartest_points(new std::vector<int>());
      int max_distance = 0;

      const int K = 1;
      for (int iter = 0; iter < max_iter; iter++) {
          pcl::IndicesPtr iter_points(new std::vector<int>());
          int iter_d = 0;

          for (int i = 0; i < n; i++) {
              int pnt = point_indexes[i];
              PointT searchPoint = mesh->points[pnt];

              std::vector<int> pointIdxNKNSearch(K);
              std::vector<float> pointNKNSquaredDistance(K);
              kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

              iter_points.push_back(mesh->points[pointIdxNKNSearch[0]]);
              iter_d += pointNKNSquaredDistance[0];
          }

          if (iter_d > max_distance) {
              max_distance = iter_d;
              fartest_points = iter_points;
          }
      }

      return fartest_points;
  }

  pcl::IndicesPtr MeshSampler::normal_sampling(PointCloudT::Ptr mesh, int num_samples) {
      int num_bins = 4;

      pcl::NormalSpaceSampling<<PointT, PointT>> sample;
      sample.setSeed(50);
      sample.setNormals(mesh);
      sample.setInputCloud(mesh);
      sample.setSample(num_samples);
      sample.setBins(num_bins, num_bins, num_bins);

      pcl::IndicesPtr indices(new std::vector<int>());
      sample.filter(*indices);

      return indices;
  }

}
