/*
 * NormalVectorAverage.cpp
 *
 *  Created on: Sep 25, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#include "foothold_finder/NormalVectorAverage.hpp"

namespace foothold_finder {

NormalVectorAverage::NormalVectorAverage()
    : referenceVector_(0.0, 0.0, 1.0)
{
  clear();
}

NormalVectorAverage::~NormalVectorAverage() {}

void NormalVectorAverage::add(const Eigen::Vector3d& vector)
{
  ++nSamples_;
  double weightPrevious = ((double) (nSamples_ - 1)) / ((double) (nSamples_));
  double weightNew = 1.0 / ((double) (nSamples_));
  Rotation newRotation;
  newRotation.setFromVectors(referenceVector_, vector);
  averageRotation_ = Rotation(weightPrevious * averageRotation_.vector() + weightNew * newRotation.vector());
}

void NormalVectorAverage::getAverage(Eigen::Vector3d& average) const
{
  average = averageRotation_.rotate(referenceVector_);
}

void NormalVectorAverage::clear()
{
  nSamples_ = 0;
  averageRotation_.setIdentity();
}

std::ostream& operator <<(std::ostream& ostream, const NormalVectorAverage& normalVectorAverage)
{
  Eigen::Vector3d average;
  normalVectorAverage.getAverage(average);
  ostream << "Number of samples: " << normalVectorAverage.nSamples_ << std::endl;
  ostream << "Average rotation: " << normalVectorAverage.averageRotation_ << std::endl;
  ostream << "Average vector: " << average.transpose() << std::endl;
  return ostream;
}

}
/* namespace foothold_finder */
