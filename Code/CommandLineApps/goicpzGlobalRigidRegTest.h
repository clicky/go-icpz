//
// Created by Murtuza Husain on 26/03/2018.
//

#ifndef GOICPZ_SUPERBUILD_GOICPZGLOBALRIGIDREGTEST_H
#define GOICPZ_SUPERBUILD_GOICPZGLOBALRIGIDREGTEST_H


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char** argv) {
    // Pre compute

    // Load moving
    PointCloudT::Ptr moving;
    if (pcl::io::loadPLYFile(argv[1], *moving) < 0) {
        std::cerr << "Error loading cloud " << argv[1] << std::endl;
    }
    std::cout << "Cloud loaded from " << argv[1] << " in " << time.toc() << "ms" << std::endl;

    // Apply transformation

    // Use KDTree to build geodesic distances



    return 0;
}

class goicpzGlobalRigidRegTest {

};


#endif //GOICPZ_SUPERBUILD_GOICPZGLOBALRIGIDREGTEST_H
