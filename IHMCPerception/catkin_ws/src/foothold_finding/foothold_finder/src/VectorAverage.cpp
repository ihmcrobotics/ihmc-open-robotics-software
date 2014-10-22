/*
 * VectorAverage.cpp
 *
 *  Created on: Sep 25, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#include "foothold_finder/VectorAverage.hpp"

namespace foothold_finder {

VectorAverage::VectorAverage()
{
  clear();
}

VectorAverage::~VectorAverage() {}

void VectorAverage::add(const Eigen::Vector3d& vector)
{
  ++nSamples_;
  double weightPrevious = ((double) (nSamples_ - 1)) / ((double) (nSamples_));
  double weightNew = 1.0 / ((double) (nSamples_));
  average_ = weightPrevious * average_ + weightNew * vector;
}

const Eigen::Vector3d& VectorAverage::getAverage() const
{
  return average_;
}

void VectorAverage::clear()
{
  nSamples_ = 0;
  average_.setZero();
}

std::ostream& operator <<(std::ostream& ostream, const VectorAverage& vectorAverage)
{
  ostream << "Number of samples: " << vectorAverage.nSamples_ << std::endl;
  ostream << "Average: " << vectorAverage.getAverage().transpose() << std::endl;
  return ostream;
}

}
/* namespace foothold_finder */
