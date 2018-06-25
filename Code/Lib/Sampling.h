namespace goicpz {

  class MeshSampler {
  public:
    pcl::IndicesPtr farthest_points(PointCloudT::Ptr mesh, int num_samples);
    pcl::IndicesPtr normal_sampling(PointCloudT::Ptr mesh, int num_samples);
  };

}
