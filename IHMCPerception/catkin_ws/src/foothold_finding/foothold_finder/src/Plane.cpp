/*
 * Plane.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <foothold_finder/Plane.hpp>

namespace foothold_finder {

Plane::Plane()
{
  normal_.setZero();
  isNormalAverageCurrent_ = false;
}

Plane::~Plane()
{
}

void Plane::set(const Eigen::Vector3d& position, const Eigen::Vector3d& normal)
{
  clear();
  averageWith(position, normal);
}

void Plane::averageWith(const Eigen::Vector3d& position, const Eigen::Vector3d& normal)
{
  positionAverage_.add(position);
  normalAverage_.add(normal);
  isNormalAverageCurrent_ = false;
}

const Eigen::Vector3d& Plane::getPosition() const
{
  return positionAverage_.getAverage();
}

const Eigen::Vector3d& Plane::getNormal()
{
  if (!isNormalAverageCurrent_) normalAverage_.getAverage(normal_);
  isNormalAverageCurrent_ = true;
  return normal_;
}

double Plane::getHeight(const Eigen::Vector2d& position)
{
  return (getNormal().dot(getPosition()) - getNormal().head(2).dot(position.head(2))) / getNormal().z();
}

void Plane::clear()
{
  normal_.setZero();
  positionAverage_.clear();
  normalAverage_.clear();
}

} /* namespace foothold_finder */

