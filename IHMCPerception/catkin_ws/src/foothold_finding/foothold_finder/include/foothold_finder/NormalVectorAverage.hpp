/*
 * NormalVectorAverage.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/rotations/RotationEigen.hpp>

// STD
#include <iostream>

namespace foothold_finder {

class NormalVectorAverage
{
 public:
  NormalVectorAverage();
  virtual ~NormalVectorAverage();

  void add(const Eigen::Vector3d& vector);

  void getAverage(Eigen::Vector3d& average) const;

  void clear();

  friend std::ostream& operator <<(std::ostream& ostream, const NormalVectorAverage& normalVectorAverage);

 private:

  typedef kindr::rotations::eigen_impl::RotationVectorAD Rotation;

  //! Number of samples.
  unsigned int nSamples_;

  Rotation averageRotation_;

  const Eigen::Vector3d referenceVector_;
};

} /* namespace foothold_finder */
