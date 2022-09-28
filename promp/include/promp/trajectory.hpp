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

#ifndef PROMP_TRAJECTORY_HPP
#define PROMP_TRAJECTORY_HPP

#include <vector>

#include <Eigen/Core>

namespace promp {

    /**
     * @brief  Class that represents a multidimensional trajectory.
     * A trajectory is described by the values at each timestep and the speed parameter.
     * speed indicates how the trajectory has been modulated, 
     * for example speed=2 means that the original trajectory had twice the timesteps.
     */
    class Trajectory {

    public:
        
        /**
         *  @\brief default constructor. Build empty trajectory.
         */
        Trajectory() = default;

        /**
         *  @\brief constructor that build a trajectory starting from data and speed
         *  @\param data Eigen::Matrix containing the raw data, each column is a different dof
         *  @\param speed speed of the original trajectory (time-scale factor: e.g., 2.0 to go from 200 time-steps to 100 time-steps)
         */
        explicit Trajectory(const Eigen::MatrixXd& data, double speed = 1.0);

        virtual ~Trajectory() = default;

        /**
         *  @\brief return number of dimensions of the trajectory
         */
        inline size_t dims() const { return _data.cols(); }

        /**
         *  @\brief return number of timesteps in the trajectory
         */
        inline size_t timesteps() const { return _data.rows(); }

        /**
         *  @\brief return the trajectory' speed
         */
        inline double speed() const { return _speed; }

        /**
         *  @\brief return the raw data as Eigen::Matrix
         */
        inline const Eigen::MatrixXd& matrix() const { return _data; }

        /**
         *  @\brief return monodimensional trajectory from the selected dimension
         *  @\param dim  dimension used to create the returned trajectory
         */
        Trajectory sub_trajectory(size_t dim) const;

        /**
         *  @\brief return  trajectory using data from the selected dimensions
         *  @\param dim  list of dimensions used to create the returned trajectory
         */
        Trajectory sub_trajectory(const std::vector<size_t>& dims) const;

        /**
         *  @\brief modulate the trajectory to the desired number of timesteps
         *  Adjust speed according to speed = this->speed() * this->timesteps() / timesteps
         *  @\param timesteps  desired number of steps in the trajectory
         */
        void modulate_in_place(size_t timesteps, bool fast = true);

        /**
         *  @\brief create a new modulated trajectory with the desired number of timesteps
         *  Adjust its speed according to speed = this->speed() * this->timesteps() / timesteps
         *  @\param timesteps  desired number of steps in the trajectory
         */
        Trajectory modulate(size_t steps, bool fast = true) const;

        /**
         *  @\brief compute the Euclidean distance between this and a second trajectory
         *  @\param other   trajectory used to compute the distance with
         *  @\param modulate  if false the distance is computed using data until the smaller trajectory lenght,
         *  if true the other trajectory is modulated to this trajectory length before computing the distance
         */
        double distance(const Trajectory& other, bool modulate = false) const;

        /**
         *  @\brief infer the speed of a trajectory starting from the raw data
         *  @\param obs_traj  data from whoch speed is inferred, comparing it to this trajectory
         *  @\param lb  lower bound for inferred speed 
         *  @\param ub  upper bound for inferred speed
         *  @\param steps  number of speeds to be tested (linspace(lb, ub, steps))
         */
        double infer_speed(const Eigen::MatrixXd& obs_traj, double lb, double ub, size_t steps) const;

    private:
        Eigen::MatrixXd create_modulated_matrix(size_t steps) const;
        Eigen::MatrixXd create_modulated_matrix_fast(size_t steps) const;

        Eigen::MatrixXd _data;
        double _speed = 1.0;
    };

} // end namespace promp

#endif // PROMP_TRAJECTORY_HPP