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

#include <promp/trajectory.hpp>

#include <iostream>
#include <stdexcept>

namespace promp 
{

    Trajectory::Trajectory(const Eigen::MatrixXd& data, double speed)
     :  _data(data),
        _speed(speed)
    { }

    Trajectory Trajectory::sub_trajectory(size_t dim) const
    {
        return Trajectory(_data.col(dim), _speed);
    }

    Trajectory Trajectory::sub_trajectory(const std::vector<size_t>& dims) const
    {
        assert(dims.size() != 0);

        Eigen::MatrixXd sub_matrix(timesteps(), dims.size());
        for(int c=0; c < dims.size(); ++c)
            sub_matrix.col(c) = _data.col(dims[c]);
        
        return Trajectory(sub_matrix, _speed);
    }

    void Trajectory::modulate_in_place(size_t steps, bool fast)
    {
        auto tmp = fast ? create_modulated_matrix_fast(steps) : create_modulated_matrix(steps);

        _data.resize(steps, this->dims());
        _data = tmp;
        _speed = _speed  * (this->timesteps() / steps);
    }

    Trajectory Trajectory::modulate(size_t steps, bool fast) const
    {
        double ratio = this->timesteps() / steps;

        if(fast)
            return Trajectory(create_modulated_matrix_fast(steps), _speed * ratio);

        return Trajectory(create_modulated_matrix(steps), _speed * ratio);
    }

    double Trajectory::distance(const Trajectory& other, bool modulate) const
    {
        assert(other.dims() == this->dims());

        if(modulate)
            return distance(other.modulate(this->timesteps()), false);

        size_t min_len = std::min(this->timesteps(), other.timesteps());
        auto diff = _data.block(0, 0, min_len, this->dims()) - other.matrix().block(0, 0, min_len, this->dims());
        return diff.colwise().norm().sum();
    }

    double Trajectory::infer_speed(const Eigen::MatrixXd& obs_traj, double lb, double ub, size_t steps) const
    {
        assert(obs_traj.cols() == this->dims());
        assert(obs_traj.rows() > 0);

        Eigen::VectorXd alphas = Eigen::VectorXd::LinSpaced(steps, lb, ub);
        
        // Find the alpha that minimizes the distance between the 
        // observed trajectory and the mean trajectory of the ProMP. 
        std::vector<double> scores(alphas.size());
        for (int i=0; i < alphas.size(); i++)
        {
            double ratio = this->speed() / alphas[i];

            // compute promp trajectory given a certain alpha
            Eigen::MatrixXd mod_traj = this->modulate(ratio * this->timesteps(), true).matrix();

            // compute derivate mean traj of promp
            int min_size = std::min(obs_traj.rows(), mod_traj.rows());

            Eigen::MatrixXd dm_traj = mod_traj.topRows(min_size-1) - mod_traj.middleRows(1, min_size-1);
            Eigen::MatrixXd do_traj = obs_traj.topRows(min_size-1) - obs_traj.middleRows(1, min_size-1);

            scores[i] = (do_traj - dm_traj).cwiseAbs().sum();
        }

        int min_distance_idx = std::min_element(scores.begin(), scores.end()) - scores.begin();
        return alphas[min_distance_idx];
    }

    Eigen::MatrixXd Trajectory::create_modulated_matrix(size_t steps) const
    {
        assert(steps !=0);
        double idx_ratio = static_cast<double>(this->timesteps()-1) / (steps-1); // interpolate over index
        
        Eigen::MatrixXd modulated(steps, dims());
        for(int r=0; r < modulated.rows(); ++r)
        {
            double pr = r * idx_ratio;

            size_t lower = std::floor(pr);
            size_t upper = std::min(static_cast<double>(_data.rows()-1), std::ceil(pr));
            
            double pre = pr - lower;

            for(int c=0; c < modulated.cols(); ++c)
                modulated(r,c) = pre * _data(upper,c) + (1-pre) * _data(lower,c);
        }

        return modulated;
    }

    Eigen::MatrixXd Trajectory::create_modulated_matrix_fast(size_t steps) const
    {
        assert(steps !=0);
        double idx_ratio = static_cast<double>(this->timesteps()-1) / (steps-1); // interpolate over index
        
        Eigen::MatrixXd modulated(steps, dims());
        for(size_t r=0; r < modulated.rows(); ++r)
            for(size_t c=0; c < modulated.cols(); ++c)
                modulated(r,c) = _data(static_cast<size_t>(r * idx_ratio), c);

        return modulated;
    }


} // end namespace promp
