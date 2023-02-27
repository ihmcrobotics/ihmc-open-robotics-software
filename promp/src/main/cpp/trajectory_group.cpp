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

#include "promp/trajectory_group.hpp"
#include <promp/io/serializer.hpp>

#include <numeric>

namespace promp
{

void TrajectoryGroup::load_trajectories(const std::vector<std::string>& files, const std::vector<size_t>& index)
{
	for(const auto& file : files)
		_trajs.push_back(io::load_trajectory(file).sub_trajectory(index));
}

void TrajectoryGroup::load_csv_trajectories(const std::vector<std::string>& files, 
	const std::vector<size_t>& index, char sep, bool skip_header)
{
	for(const auto& file : files)
	{
		io::CSVReader reader(file, sep, skip_header);		
		_trajs.push_back(Trajectory(reader.get_data(), 1.0).sub_trajectory(index));
	}
}

void TrajectoryGroup::load_csv_trajectories(const std::vector<std::string>& files, 
	const std::vector<std::string>& index, char sep)
{
	for(const auto& file : files)
	{
		io::CSVReader reader(file, sep, true);	
		_trajs.push_back(Trajectory(reader.get_data_dofs(index, reader.get_data()), 1.0));
	}
}

size_t TrajectoryGroup::normalize_length()
{
	if(_trajs.empty()) 
		throw std::logic_error("Cannot TrajectoryGroup::standardize_dims() before loading trajectories.");
	
	auto sum_all = [](size_t sum, const Trajectory& tr) { return sum + tr.timesteps(); };
	size_t mean_len = std::accumulate(_trajs.begin(), _trajs.end(), 0, sum_all) / _trajs.size();
	
	for(auto& trajectory : _trajs)
		trajectory.modulate_in_place(mean_len);

	return mean_len;
}

void TrajectoryGroup::normalize_length(size_t len)
{
	if(_trajs.empty()) 
		throw std::logic_error("Cannot TrajectoryGroup::standardize_dims() before loading trajectories.");

	for(auto& trajectory : _trajs)
		trajectory.modulate_in_place(len);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> TrajectoryGroup::standardize_dims()
{
	if(_trajs.empty()) 
		throw std::logic_error("Cannot TrajectoryGroup::standardize_dims() before loading trajectories.");
	
	Eigen::VectorXd dofs_mean = Eigen::VectorXd::Zero(_trajs[0].dims());
	Eigen::VectorXd dofs_std = Eigen::VectorXd::Zero(_trajs[0].dims());

	for(const auto& trajectory : _trajs)
		dofs_mean += trajectory.matrix().colwise().mean();
	dofs_mean = (1.0 / _trajs.size()) * dofs_mean;

	// 'dofs' centering all trajectories
	for(int i=0; i < _trajs.size(); ++i)
		_trajs[i] = Trajectory((_trajs[i].matrix().rowwise() - dofs_mean.transpose()), _trajs[i].speed());
	
	double elem_count = 0;
	for(const auto& trajectory : _trajs)
	{
		dofs_std += trajectory.matrix().array().square().matrix().colwise().sum();
		elem_count += trajectory.timesteps();
	}
	
	dofs_std = ((1.0 / elem_count) * dofs_std).cwiseSqrt();

	// 'dofs' standardiazation wrt all trajectories
	for(int i=0; i < _trajs.size(); ++i)
		_trajs[i] = Trajectory(_trajs[i].matrix().array().rowwise() * dofs_std.transpose().array(), _trajs[i].speed());

	return std::make_pair(dofs_mean, dofs_std);
}

} // end namespace promp