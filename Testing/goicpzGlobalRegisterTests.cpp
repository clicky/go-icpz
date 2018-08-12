//
// Created by Murtuza Husain on 08/08/2018.
//

#include "goicpzGlobalRegisterTests.h"


TEST_CASE("Sort Vector Test", "[globalregister]") {
    Eigen::VectorXf v(5);
    v << 3.1, 3.01, 7, 4, 6;

    std::vector<float> sorted;
    std::vector<int> idx;

    goicpz::sort_desc(v, sorted, idx);

    const int sz = 5;
    REQUIRE(sorted.size() == sz);
    REQUIRE(idx.size() == sz);

    float expected_val[sz] = { 7, 6, 4, 3.1, 3.01 };
    int expected_idx[sz] = { 2, 4, 3, 0, 1 };

    for (int i=0; i<sz; i++) {
        REQUIRE(expected_val[i] == sorted[i]);
        REQUIRE(expected_idx[i] == idx[i]);
    }
}