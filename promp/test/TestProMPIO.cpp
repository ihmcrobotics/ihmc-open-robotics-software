//
// Copyright (c) 2022, INRIA
//
// This file is part of promp.
// promp is free software: you can redistribute it and/or modify it under the terms of 
// the GNU Lesser General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.
// promp is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or 
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for 
// more details.
// You should have received a copy of the GNU Lesser General Public License along with 
// promp. If not, see <https://www.gnu.org/licenses/>. 
//

#include <gtest/gtest.h>

#include <promp/io/serializer.hpp>

TEST(IO, prompSave)
{
    std::string fileName = "promp.txt";
    std::string folderPath = "../../etc/test/";
    auto promp = promp::io::load_promp(folderPath + fileName);
    promp::io::save_promp(folderPath + "__" + fileName, promp);
    auto savedPromp = promp::io::load_promp(folderPath + "__" + fileName);

    ASSERT_EQ(promp.get_std_bf(), savedPromp.get_std_bf());
    ASSERT_EQ(promp.get_dims(), savedPromp.get_dims());
    ASSERT_EQ(promp.get_n_samples(), savedPromp.get_n_samples());
    ASSERT_EQ(promp.get_weights(), savedPromp.get_weights());
    ASSERT_EQ(promp.get_covariance(), savedPromp.get_covariance());
}

TEST(IO, trajectorySave)
{
    size_t timesteps = 100;
    size_t dims = 5;
    double speed = 1.2;

    Eigen::MatrixXd random(timesteps, dims);
    random.setRandom();

    promp::Trajectory original(random, speed);
    promp::io::save_trajectory("../../etc/test/trajectory_saved.txt", original);
    auto loaded = promp::io::load_trajectory("../../etc/test/trajectory_saved.txt");

    ASSERT_EQ(original.speed(), loaded.speed());
    ASSERT_EQ(original.timesteps(), loaded.timesteps());
    ASSERT_EQ(original.dims(), loaded.dims());
    EXPECT_LE((original.matrix() - loaded.matrix()).maxCoeff(), 1e-15);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}