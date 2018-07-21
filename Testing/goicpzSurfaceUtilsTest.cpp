//
// Created by Murtuza Husain on 16/07/2018.
//

#include <math.h>
#include "goicpzSurfaceUtilsTest.h"
#include "SurfaceRegister.h"

TEST_CASE("Unique Indices", "[surfaceregister]") {
    pcl::IndicesPtr sample1(new std::vector<int>());
    // Missing 0
    sample1->push_back(1);
    sample1->push_back(2);
    sample1->push_back(3);
    sample1->push_back(4);
    sample1->push_back(5);

    pcl::IndicesPtr sample2(new std::vector<int>());
    sample1->push_back(5); // dupe
    sample1->push_back(6);
    sample1->push_back(7);
    // Missing 8, 9
    sample1->push_back(1); // dupe
    sample1->push_back(10);

    goicpz::SurfaceRegister reg;
    pcl::IndicesPtr res = reg.unique_indices(sample1, sample2);

    REQUIRE(res->size() == 8);

    bool flags [11] = { false, false, false, false, false, false, false, false, false, false, false };
    for (int i = 0; i < 8; i++) {
        flags[(*res)[i]] = true;
    }

    REQUIRE_FALSE(flags[0]);
    for (int i = 1; i < 8; i++) {
        REQUIRE(flags[i]);
    }
    REQUIRE_FALSE(flags[8]);
    REQUIRE_FALSE(flags[9]);
    REQUIRE(flags[10]);
}

TEST_CASE("PLY loader", "[surfaceutils]") {
    PointCloudT::Ptr cloud(new PointCloudT());
    //load_mesh("/Users/murtuza/dev/CPP/dissertation/go-icpz/Testing/Data/bun000.ply", cloud);
    load_mesh("/Users/murtuza/dev/Matlab/globalShapeMatching/output/meshForTOLDI.ply", cloud);

    int sz = cloud->points.size();
    REQUIRE(sz > 0);

    int checks [3] = { 0, sz/2, sz-1 };
    for (int i = 0; i < 3; i++) {
        float x = cloud->points[checks[i]].x;
        float y = cloud->points[checks[i]].y;
        float z = cloud->points[checks[i]].z;

        REQUIRE_FALSE(isnan(x));
        REQUIRE_FALSE(isnan(y));
        REQUIRE_FALSE(isnan(z));
    }
}

TEST_CASE("Farthest Point Sample", "[surfaceregister]") {
    goicpz::SurfaceRegister reg;
    reg.load_surface("/Users/murtuza/dev/Matlab/globalShapeMatching/output/meshForTOLDI.ply");

    int sz = 100;
    pcl::IndicesPtr sample = reg.farthest_point_sample(sz);

    REQUIRE(sample->size() == sz);
}

TEST_CASE("Normal Space Sample", "[surfaceregister]") {
    goicpz::SurfaceRegister reg;
    reg.load_surface("/Users/murtuza/dev/Matlab/globalShapeMatching/output/meshForTOLDI.ply");

    int sz = 1300;
    pcl::IndicesPtr sample = reg.normal_space_sample(sz);

    REQUIRE(sample->size() == sz);
}