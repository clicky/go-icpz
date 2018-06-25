typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace goicpz {

  class RigidIcp {
  private:
    PointCloudT::Ptr target;
    PointCloudT::Ptr reconstructed;
  public:
    void load_mesh(std::String path);
    pcl::IndicesPtr select_features(int num_features);
  }

}
