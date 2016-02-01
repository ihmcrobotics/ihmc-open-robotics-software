/*
 * FootholdFinderTest.cpp
 *
 *  Created on: Sep 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_lib/FootholdFinder.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/rotations/RotationEigen.hpp>

using namespace std;
using namespace Eigen;
using namespace grid_map_lib;

TEST(matchToSurfaceNormal, Standard)
{
//  Eigen::Vector3d normal;
//  normal << 0.0, 0.0, 1.0;
//  kindr::rotations::eigen_impl::EulerAnglesZyxPD rpy(0.0, 0.0, 0.0);
//  kindr::rotations::eigen_impl::RotationQuaternionPD rotation(rpy);
//  matchToSurfaceNormal(normal, rotation);
//  rpy(rotation);
//  rpy = rpy.getUnique();
//  EXPECT_EQ(0.0, rpy.roll() / M_PI * 180.0);
//  EXPECT_EQ(0.0, rpy.pitch() / M_PI * 180.0);
//  EXPECT_EQ(0.0, rpy.yaw() / M_PI * 180.0);

//  normal << 0.0, 0.707, 0.707;
//  kindr::rotations::eigen_impl::EulerAnglesZyxPD rpy(-M_PI_2, 0.0, 0.0);
//  std::cout << "roll: " << rpy.roll() / M_PI * 180.0 << endl;
//  std::cout << "pitch: " << rpy.pitch() / M_PI * 180.0 << endl;
//  std::cout << "yaw: " << rpy.yaw() / M_PI * 180.0 << endl;
//  rotation(rpy);
//  std::cout << "normal: " << normal.transpose() << std::endl;
//  std::cout << "rotation: " << rotation << std::endl;
//  matchToSurfaceNormal(normal, rotation);
//  std::cout << "rotation after: " << rotation << std::endl;
//  rpy(rotation);
//  rpy = rpy.getUnique();
//  std::cout << "roll: " << rpy.roll() / M_PI * 180.0 << endl;
//  std::cout << "pitch: " << rpy.pitch() / M_PI * 180.0 << endl;
//  std::cout << "yaw: " << rpy.yaw() / M_PI * 180.0 << endl;

//  Eigen::Vector3d normal(0.0, 0.70710678118, 0.70710678118);
//  Rotation rotation(kindr::rotations::eigen_impl::EulerAnglesYprPD(-90.0 / 180.0 * M_PI, 0.0, -22.5 / 180.0 * M_PI));
//  matchToSurfaceNormal(normal, rotation);

  // TODO Make this a unit test.
//  Eigen::Vector3d referenceTest = Eigen::Vector3d::UnitZ();
//  referenceTest = rotation.inverseRotate(referenceTest);
//  std::cout << "Reference test: " << referenceTest.transpose() << std::endl;
//  double differenceTest = acos(referenceTest.dot(normal) / (referenceTest.norm() * referenceTest.norm())) / M_PI * 180.0;
//  std::cout << "Angle [deg] between normal and reference test: " << differenceTest << std::endl;

}
