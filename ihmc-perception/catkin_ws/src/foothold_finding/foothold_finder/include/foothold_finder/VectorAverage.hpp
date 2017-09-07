/*
 * VectorAverage.hpp
 *
 *  Created on: Sep 25, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// Eigen
#include <Eigen/Core>

// STD
#include <iostream>

namespace foothold_finder {

class VectorAverage
{
 public:
  VectorAverage();
  virtual ~VectorAverage();

  void add(const Eigen::Vector3d& vector);

  const Eigen::Vector3d& getAverage() const;

  void clear();

  friend std::ostream& operator <<(std::ostream& ostream, const VectorAverage& vectorAverage);

 private:

  //! Number of samples.
  unsigned int nSamples_;

  //! Average vector.
  Eigen::Vector3d average_;
};

} /* namespace foothold_finder */
