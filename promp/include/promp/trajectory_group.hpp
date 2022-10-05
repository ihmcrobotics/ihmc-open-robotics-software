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

#ifndef PROMP_TRAJECTORY_GROUP_HPP
#define PROMP_TRAJECTORY_GROUP_HPP

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <promp/io/csv_reader.hpp>
#include <promp/trajectory.hpp>

namespace promp {

    /**
     * @brief      Class for handling data from multiple trajectories.
     * Data from each trajectory should be stored in an individual file, 
     * files can be generated from serializer or be in .csv format.
     * Each column within .csv file represents a trajectory.
     * All .csv files must have trajectories(columns) in the same sequence (column-wise)
     */
    class TrajectoryGroup {

    public:
        
        /**
         *  @\brief default constructor: create an empty trajectory group
         */
        TrajectoryGroup() = default;

        /**
         *  @\brief    load trajectories from list of files (formatted as generated from io/serializer). 
         *	@\param	files of files.
         *	@\param	index list of indexes representing dofs to keep.
         */
        void load_trajectories(const std::vector<std::string>& files, const std::vector<size_t>& index);

        /**
         *  @\brief    load trajectories from list of .csv files. 
         *	@\param	files of files.
         *	@\param	index list of indexes representing dofs to keep.
         *	@\param	sep values separator.
         *	@\param	skip_header if true skip first line.
         */
        void load_csv_trajectories(const std::vector<std::string>& files,
            const std::vector<size_t>& index, char sep = ',', bool skip_header = false);

        /**
         *  @\brief    load trajectories from list of .csv files. 
         *	@\param	files of files.
         *	@\param	cols list of columns (dofs) to keep.
         *	@\param	sep values separator.
         */
        void load_csv_trajectories(const std::vector<std::string>& files,
            const std::vector<std::string>& cols, char sep = ',');

        /**
         *   @\brief    Normalize all trajectories to the mean length (number of timesteps)
         */
        size_t normalize_length();

        /**
         *  @\brief     Normalize all trajectories to the same desired length
         *	@\param	len desired length
         */
        void normalize_length(size_t len);

        /**
         *   @\brief    standardize each dof among the trajectories
         */
        std::pair<Eigen::VectorXd, Eigen::VectorXd> standardize_dims();

        /**
         * @\brief	return the vector of trajectories
         */
        const std::vector<Trajectory>& trajectories() const { return _trajs; }

    private:

        //! vector of loaded trajectories
        std::vector<Trajectory> _trajs;
    };

} // end namespace promp

#endif // PROMP_TRAJECTORY_GROUP_HPP