/*
 * Plane.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "foothold_finder/VectorAverage.hpp"
#include "foothold_finder/NormalVectorAverage.hpp"

// Eigen
#include <Eigen/Core>

namespace foothold_finder {

/*!
 * Plane parameterization and handling of averages.
 */
class Plane
{
 public:
  /*!
   * Constructor.
   */
  Plane();

  /*!
   * Destructor
   */
  virtual ~Plane();

  /*!
   * Set the plane parameters.
   * @param position
   * @param normal
   */
  void set(const Eigen::Vector3d& position, const Eigen::Vector3d& normal);

  /*!
   * Average the current plane parameters with new values.
   * @param position the position to be added to the average.
   * @param normal the normal to be added to the average.
   */
  void averageWith(const Eigen::Vector3d& position, const Eigen::Vector3d& normal);

  /*!
   * Get position of the plane.
   * @return the position (centroid) of the plane.
   */
  const Eigen::Vector3d& getPosition() const;

  /*!
   * Get surface normal of the plane.
   * @return the surface normal.
   */
  const Eigen::Vector3d& getNormal();

  /*!
   * Get height (z-coordinate) at position.
   * @param[in] position the position to get the height at.
   * @return the height.
   */
  double getHeight(const Eigen::Vector2d& position);

  /*!
   * Clears the data and averages.
   */
  void clear();

 private:

  //! Position of the centroid.
  VectorAverage positionAverage_;

  //! Surface normal.
  NormalVectorAverage normalAverage_;

  //! If normal average is up to date.
  bool isNormalAverageCurrent_;

  //! Surface normal for caching reasons.
  Eigen::Vector3d normal_;
};

} /* namespace foothold_finder */
