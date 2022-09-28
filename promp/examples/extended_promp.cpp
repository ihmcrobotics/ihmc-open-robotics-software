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

	auto mean_std_pair = std::make_pair(
    	Eigen::VectorXd::Zero(dofs.size()),
	    Eigen::VectorXd::Ones(dofs.size()));
	
	/// initialize promp object with number of basis functions and std as arguments.
	int n_rbf = 20;
	promp::ProMP m_promp(trajectory_group, n_rbf);
	auto aasd = std::chrono::high_resolution_clock::now();
	
	auto a = std::chrono::high_resolution_clock::now();
	Eigen::MatrixXd vect = m_promp.generate_trajectory();
	auto b = std::chrono::high_resolution_clock::now();
	Eigen::MatrixXd stdTraj = m_promp.gen_traj_std_dev();
	auto e = std::chrono::high_resolution_clock::now();
	auto traj_covariance = m_promp.generate_trajectory_covariance();
	auto s = std::chrono::high_resolution_clock::now();
	// Eigen::MatrixXd vect2 = m_promp.generate_trajectory(127);
	auto i = std::chrono::high_resolution_clock::now();
	// Eigen::MatrixXd mTraj = m_promp.gen_traj_std_dev(20);
	auto h = std::chrono::high_resolution_clock::now();

	// for(int i = 0; i < dofs.size(); ++i)
	// {
	// 	Eigen::VectorXd test(stdTraj.rows());
	// 	for(int t=0; t < stdTraj.rows(); ++t)
	// 		test(t) = traj_covariance[t](i,i);
	// 	std::cerr << "max difference with std_dev in dimension " << i 
	// 		<< ": " << (test.cwiseSqrt() - stdTraj.col(i)).maxCoeff() << std::endl;
	// }

	{
		size_t ntraj = trajectory_group.trajectories().size();
		size_t ndims = trajectory_group.trajectories()[0].dims();
		Eigen::MatrixXd all_data(t_len, ndims * ntraj);
		size_t it = 0;
		for(const auto& t : trajectory_group.trajectories())
			all_data.block(0, ndims * (it++), t_len, ndims) = t.matrix();

		std::ofstream out("trajectories.csv");
		out << all_data.format(csv_format);
		out.close();
	}

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
	

	promp::Trajectory traj_original(vect, 1.0);

	double speed_mod = 1.75;
	size_t timestep_original = traj_original.timesteps();
	promp::Trajectory traj_modulated = traj_original.modulate(timestep_original / 1.75);

	double infer_speed = 0; // traj_original.infer_speed(traj_modulated.matrix(), 0.25, 4.0, 20);
	std::cerr << "inferred speed for modulated trajectory: " << infer_speed << std::endl;

	std::cerr << "traj: " << std::chrono::duration_cast<std::chrono::microseconds>(b-a).count() << " mu s" << std::endl;
	std::cerr << "std: " << std::chrono::duration_cast<std::chrono::microseconds>(e-b).count() << " mu s" << std::endl;
	std::cerr << "traj cov: " << std::chrono::duration_cast<std::chrono::microseconds>(s-e).count() << " mu s" << std::endl;
	//std::cerr << "traj std int: " << std::chrono::duration_cast<std::chrono::microseconds>(i-s).count() << " mu s" << std::endl;

	//std::cerr << "traj_1 " << vect.rows() << " " << vect.cols() << std::endl;
	//std::cerr << "traj_2 " << mTraj.rows() << " " << mTraj.cols() << std::endl;
	
	Eigen::VectorXd via_point_1 = trajectory_group.trajectories()[0].matrix().row(0); // Eigen::VectorXd::Zero(dofs.size());
	Eigen::VectorXd via_point_2 = trajectory_group.trajectories()[0].matrix().row(200); // -0.01 * Eigen::VectorXd::Ones(dofs.size());
	Eigen::VectorXd via_point_3 = trajectory_group.trajectories()[0].matrix().row(100); // -0.01 * Eigen::VectorXd::Ones(dofs.size());
	Eigen::MatrixXd via_point_std = 1e-5 * Eigen::MatrixXd::Identity(dofs.size(), dofs.size()); // if normalized, use smaller value

	via_point_std.diagonal() = mean_std_pair.second.cwiseProduct(via_point_std.diagonal());
	// std::cerr << "via point std: " << via_point_std << std::endl;

	std::vector<std::tuple<int, Eigen::VectorXd, Eigen::MatrixXd>> via_points = {
		std::make_tuple<>(200, via_point_2, via_point_std),
		std::make_tuple<>(0, via_point_1, via_point_std),
		std::make_tuple<>(100, via_point_3, via_point_std)
	};

	m_promp.condition_via_points(via_points); // multiple case
	//m_promp.condition_via_point(100, via_point_1, via_point_std); // single case



	Eigen::VectorXd www = m_promp.get_weights();
	Eigen::VectorXd aaa = m_promp.get_upper_weights(1.0);
	Eigen::VectorXd aav = m_promp.get_lower_weights(1.0);

	// std::cerr << aaa.size() << std::endl;
	// std::cerr << aav.size() << std::endl;
	// std::cerr << "weights\n"<< www.transpose() << std::endl;
	// std::cerr << "upper weights\n"<< aaa.transpose() << std::endl;
	// std::cerr << "lower weights\n"<< aav.transpose() << std::endl;

	auto print_has_nans = [=](const std::string& name, const Eigen::MatrixXd& m) {
		std::cerr << name << ((m.array() == m.array()).all() ? " has only numericals\n" : " contains NaN values\n");
	};

	print_has_nans("trajectory", vect);
	print_has_nans("std_traj", stdTraj);	
	print_has_nans("weights", aaa);
	print_has_nans("upper_weights", www);
	print_has_nans("upper_weights", aaa);


	Eigen::MatrixXd conditioned_traj = m_promp.generate_trajectory();
	Eigen::MatrixXd conditioned_std_traj = m_promp.gen_traj_std_dev();

	{
		std::ofstream out("generated_conditioned.csv");
		out << conditioned_traj.format(csv_format);
		out.close();
	}

	{
		std::ofstream out("variance_conditioned.csv");
		out << conditioned_std_traj.format(csv_format);
		out.close();
	}

	return 0;
}