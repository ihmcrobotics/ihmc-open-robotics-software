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
	std::string file_path = "../etc/demos/Reaching1/";
	for (int i=0; i<10; i++)
	    file_list.push_back(file_path + "pr" + std::to_string(i+1) + ".csv");

	// data handle object which takes indeces of trajectories in .csv file and also the file list
	std::vector<size_t> dofs = {0, 1, 2};
	promp::TrajectoryGroup trajectory_group;
	trajectory_group.load_csv_trajectories(file_list, dofs);
	size_t t_len = trajectory_group.normalize_length();

	std::vector<std::string> file_list_test;
    std::string file_path_test = "../etc/demos/Reaching2/";
    for (int i=0; i<6; i++)
    	    file_list_test.push_back(file_path_test + "pr" + std::to_string(i+1) + ".csv");
    promp::TrajectoryGroup trajectory_group_test;
    trajectory_group_test.load_csv_trajectories(file_list_test, dofs);

	auto mean_std_pair = std::make_pair(
    	Eigen::VectorXd::Zero(dofs.size()),
	    Eigen::VectorXd::Ones(dofs.size()));
	
	/// initialize promp object with number of basis functions and std as arguments.
	int n_rbf = 20;
	promp::ProMP m_promp(trajectory_group, n_rbf);

	Eigen::MatrixXd vect = m_promp.generate_trajectory();
	Eigen::MatrixXd stdTraj = m_promp.gen_traj_std_dev();
	auto traj_covariance = m_promp.generate_trajectory_covariance();

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

    // infer speed of promp
	std::vector<promp::Trajectory> demo_test_trajectories = trajectory_group_test.trajectories();
	promp::Trajectory demo_test_trajectory = demo_test_trajectories[0]; //select demo test trajectory to observe
    Eigen::MatrixXd observed_trajectory;
    observed_trajectory.resize(demo_test_trajectory.timesteps()/3,dofs.size()); //observe a third of the trajectory
    for (int i=0; i<observed_trajectory.rows(); i++)
        for (int j=0; j<observed_trajectory.cols(); j++)
            observed_trajectory(i, j) = (demo_test_trajectories[0].matrix())(i, j);
	int demo = promp::Trajectory::infer_closest_trajectory(observed_trajectory,demo_test_trajectories);
	std::cerr << "inferred closest demo to current observation: " << (demo+1) << std::endl;

    double infer_speed = demo_test_trajectories[demo].infer_speed(observed_trajectory, 0.25, 4.0, 30);
    std::cerr << "inferred speed for modulated trajectory: " << infer_speed << std::endl;
    std::cerr << "timesteps actual demo: " << demo_test_trajectory.timesteps() << std::endl;
    promp::Trajectory demo_test_trajectory_modulated = demo_test_trajectory.modulate(demo_test_trajectory.timesteps()/infer_speed);
    std::cerr << "timesteps modulated trajectory: " << demo_test_trajectory_modulated.timesteps() << std::endl;

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