import os
import promp
import time
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":

	os.makedirs("output", exist_ok=True)

	# list of strings containing file names of all .csv files (each .csv file contains data from a single demonstration)
	file_path = "../etc/demos/PutasideBox/"
	file_list = list()
	file_list.append(file_path + "p1.csv")
	file_list.append(file_path + "p2.csv")
	file_list.append(file_path + "p3.csv")
	file_list.append(file_path + "p4.csv")
	file_list.append(file_path + "p5.csv")
	file_list.append(file_path + "p6.csv")
	file_list.append(file_path + "p7.csv")
	file_list.append(file_path + "p8.csv")
	file_list.append(file_path + "p9.csv")
	file_list.append(file_path + "p10.csv")

	# data handle object which takes indeces of trajectories in .csv file and also the file list
	dofs = [0, 1, 2]
	n_dofs = len(dofs)
	trajectory_group = promp.TrajectoryGroup()
	trajectory_group.load_csv_trajectories(file_list, dofs)
	timesteps = trajectory_group.normalize_length()

	mean_std_pair = ( np.zeros((n_dofs,1), np.float64), np.ones((n_dofs, 1), np.float64) )
	#  mean_std_pair = trajectory_group.standardize_dims()

	# initialize promp object with number of basis functions and std as arguments.
	n_rbf = 20
	m_promp = promp.ProMP(trajectory_group.trajectories(), n_rbf, 1.0 / (n_rbf*n_rbf))
	
	a = time.time()
	gen_traj = m_promp.generate_trajectory()
	b = time.time()
	gen_traj_stddev = m_promp.gen_traj_std_dev()
	e = time.time()
	gen_traj_covariance = m_promp.generate_trajectory_covariance()
	s = time.time()
	#  gen_traj2 = m_promp.generate_trajectory(127)
	i = time.time()
	#  traj_std_dev = m_promp.gen_traj_std_dev(20)
	h = time.time()


	all_example_trajectories = np.concatenate([x.matrix() for x in trajectory_group.trajectories()], 1)
	np.savetxt("output/trajectories.csv", all_example_trajectories, delimiter=',')
	np.savetxt("output/generated.csv", gen_traj, delimiter=',')
	np.savetxt("output/variance.csv", gen_traj_stddev, delimiter=',')
	np.savetxt("output/covariance.csv", np.concatenate(gen_traj_covariance, 0), delimiter=',')

	traj_original = promp.Trajectory(gen_traj, 1.0)

	speed_mod = 1.75
	timestep_original = traj_original.timesteps()
	traj_modulated = traj_original.modulate(int(timestep_original / 1.75))

	infer_speed = traj_original.infer_speed(traj_modulated.matrix(), 0.25, 4.0, 20)
	print("inferred speed for modulated trajectory: {:.5f}".format(infer_speed) )

	print("trajectory generation time: {:.3f} microseconds".format((b-a) * 1000 * 1000) )
	print("std dev generation time: {:.3f} microseconds".format((e-b) * 1000 * 1000) )
	print("covariance generation time: {:.3f} microseconds".format( (s-e) * 1000 * 1000) )
	# print("traj std int: ", (i-s) * 1000 * 1000, " mu s" )

	# print("traj_1 ", gen_traj.rows(), " ", gen_traj.cols() )
	# print("traj_2 ", traj_std_dev.rows(), " ", traj_std_dev.cols() )
	
	via_point_1 = trajectory_group.trajectories()[0].matrix()[0,:]
	via_point_2 = trajectory_group.trajectories()[0].matrix()[200,:]
	via_point_3 = trajectory_group.trajectories()[0].matrix()[100,:]
	via_point_std = 1e-5 * np.identity(n_dofs) # if normalized, use smaller value

	# via_point_std.diagonal() = mean_std_pair.second.cwiseProduct(via_point_std.diagonal())
	#  std::cerr << "via point std: " << via_point_std << std::endl

	via_points = list()
	via_points.append( (0, via_point_1, via_point_std) )
	via_points.append( (200, via_point_2, via_point_std) )
	via_points.append( (100, via_point_3, via_point_std) )

	m_promp.condition_via_points(via_points); #  multiple case
	# m_promp.condition_via_point(100, via_point_1, via_point_std); #  single case

	www = m_promp.get_weights()
	aaa = m_promp.get_upper_weights(1.0)
	aav = m_promp.get_lower_weights(1.0)

	conditioned_traj = m_promp.generate_trajectory()
	conditioned_std_traj = m_promp.gen_traj_std_dev()

	np.savetxt("output/generated_conditioned.csv", conditioned_traj, delimiter=',')
	np.savetxt("output/variance_conditioned.csv", conditioned_std_traj, delimiter=',')

	for d in range(n_dofs):
		plt.subplot()
		for t in trajectory_group.trajectories():
			plt.plot(t.matrix()[:,d],"--")
		plt.plot(gen_traj[:,d], 'b', label="mean")
		plt.plot(conditioned_traj[:,d], 'r', label="conditioned")

		lower_bound = gen_traj[:,d] - gen_traj_stddev[:,d]
		upper_bound = gen_traj[:,d] + gen_traj_stddev[:,d]
		plt.fill_between(np.linspace(0,timesteps-1, timesteps), lower_bound, upper_bound, color='blue', alpha=0.2)

		lower_bound = conditioned_traj[:,d] - conditioned_std_traj[:,d]
		upper_bound = conditioned_traj[:,d] + conditioned_std_traj[:,d]
		plt.fill_between(np.linspace(0,timesteps-1, timesteps), lower_bound, upper_bound, color='red', alpha=0.2)

		plt.title(file_path + " - dof {}".format(d))
		plt.grid()
		plt.legend()
		plt.savefig("output/traj_{}.png".format(d))
		plt.close()

	exit()
