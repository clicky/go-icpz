//
// Created by Murtuza Husain on 16/07/2018.
//

#include "goicpzSurfaceUtilsTest.h"

TEST_CASE("Unique Indices", "[surfaceutils]") {
    pcl::IndicesPtr sample1(new std::vector<int>());
    sample1->push_back(1);
    sample1->push_back(2);
    sample1->push_back(3);
    sample1->push_back(4);
    sample1->push_back(5);

    pcl::IndicesPtr sample2(new std::vector<int>());
    sample1->push_back(5); // dupe
    sample1->push_back(6);
    sample1->push_back(7);
    sample1->push_back(1); // dupe
    sample1->push_back(10);

    pcl::IndicesPtr res = unique_indices(sample1, sample2);

    REQUIRE(res->size() == 8);
}