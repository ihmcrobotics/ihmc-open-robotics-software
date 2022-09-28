#include <promp/promp.hpp>
#include <promp/trajectory.hpp>

#include <fstream>
#include <chrono>
#include <vector>
#include <string>

#include <cassert>

Eigen::IOFormat csv_format(Eigen::FullPrecision, 0, ",");

int main()
{
	//vector of strings containing file names of all .csv files (each .csv file contains data from a single demonstration)
	std::vector<std::string> file_list;
	std::string file_path = "../etc/demos/PutasideBox/";
	file_list.push_back(file_path + "p1.csv");
	file_list.push_back(file_path + "p2.csv");
	file_list.push_back(file_path + "p3.csv");
	file_list.push_back(file_path + "p4.csv");
	file_list.push_back(file_path + "p5.csv");
	file_list.push_back(file_path + "p6.csv");
	file_list.push_back(file_path + "p7.csv");
	file_list.push_back(file_path + "p8.csv");
	file_list.push_back(file_path + "p9.csv");
	file_list.push_back(file_path + "p10.csv");

	// data handle object which takes indeces of trajectories in .csv file and also the file list
	std::vector<size_t> dofs = {0, 1, 2};
	promp::TrajectoryGroup trajectory_group;
	trajectory_group.load_csv_trajectories(file_list, dofs);
	size_t t_len = trajectory_group.normalize_length();

	/// initialize promp object with number of basis functions and std as arguments.
	int n_rbf = 20;
	promp::ProMP m_promp(trajectory_group, n_rbf);
	Eigen::MatrixXd vect = m_promp.generate_trajectory();
	Eigen::MatrixXd stdTraj = m_promp.gen_traj_std_dev();
	auto traj_covariance = m_promp.generate_trajectory_covariance();
	
	{
		std::ofstream out("generated.csv");
		out << vect.format(csv_format);
		out.close();
	}

	{
		std::ofstream out("variance.csv");
		out << stdTraj.format(csv_format);
		out.close();
	}

	{
		std::ofstream out("covariance.csv");
		for(const auto& cov : traj_covariance)
			out << cov.format(csv_format) << std::endl;
		out.close();
	}
	

	
	return 0;
}