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

#include "promp/promp.hpp"

#include <iterator>
#include <cassert>
#include <stdio.h>
#include <math.h>
#include <random>
#include <cmath>

class RepeatBlockDiagonalMatrix
{
public:
	RepeatBlockDiagonalMatrix(const Eigen::MatrixXd& mtx, size_t repeat_n)
		:	_mtx(mtx),
		_rep(repeat_n)
	{

	}

	inline RepeatBlockDiagonalMatrix transpose()
	{
		return RepeatBlockDiagonalMatrix(_mtx.transpose(), _rep);
	}

	inline RepeatBlockDiagonalMatrix inverse()
	{
		return RepeatBlockDiagonalMatrix(_mtx.inverse(), _rep);
	}

	inline size_t rows() const { return _mtx.rows() * _rep; }
	inline size_t cols() const { return _mtx.cols() * _rep; }
	inline size_t block_rows() const { return _mtx.rows(); }
	inline size_t block_cols() const { return _mtx.cols(); }

	inline size_t reps() const { return _rep; }

	Eigen::MatrixXd full_matrix() const
	{
		size_t rows = _mtx.rows();
		size_t cols = _mtx.cols();
		Eigen::MatrixXd full = Eigen::MatrixXd::Zero(rows * _rep, cols * _rep);
		for(int r=0; r < _rep; ++r)
			full.block(r * rows, r * cols, rows, cols) = _mtx;
		return full;
	}

	inline const Eigen::MatrixXd& matrix_block() const { return _mtx; };

private:

	Eigen::MatrixXd _mtx;
	size_t _rep;
};

inline Eigen::MatrixXd operator*(const Eigen::MatrixXd& m, const RepeatBlockDiagonalMatrix& mbd)
{
	assert(m.cols() == mbd.rows());

	size_t b_rows = mbd.block_rows();
	size_t b_cols = mbd.block_cols();

	Eigen::MatrixXd res(m.rows(), mbd.cols());
	for(int i=0; i < mbd.reps(); ++i)
		res.block(0, i * b_cols, m.rows(), b_cols) = m.block(0, i * b_rows, m.rows(), b_rows) * mbd.matrix_block();
	return res;

	// TODO test: split in more block could speed up
}

inline Eigen::MatrixXd operator*(const RepeatBlockDiagonalMatrix& mbd, const Eigen::MatrixXd& m)
{
	assert(mbd.cols() == m.rows());

	size_t b_rows = mbd.block_rows();
	size_t b_cols = mbd.block_cols();

	Eigen::MatrixXd res(mbd.rows(), m.cols());
	for(int i=0; i < mbd.reps(); ++i)
		res.block(i * b_rows, 0, b_rows, m.cols()) = mbd.matrix_block() * m.block(i * b_cols, 0, b_cols, m.cols());
	return res;

	// TODO test: split in more block could speed up
}

namespace promp
{

void ProMP::init(const std::vector<Trajectory>& data, double std_bf)
{
	assert(data.size() > 0);
	_std_bf = std_bf <= 0 ? 1.0 / (_num_bf * _num_bf) : std_bf;
	_num_demos = data.size();
	_dims = data[0].dims();
	_s = data[0].timesteps(); 

	_phase = compute_phase();
	_phi = generate_basis_function(_phase);
	train(data);
} 

ProMP::ProMP(const Eigen::VectorXd& w, const Eigen::MatrixXd& cov_w, double std_bf, int n_sample, size_t dims, double time_mod) :
	_mean_w(w),
	_cov_w(cov_w),
	_num_bf(w.size() / dims),
	_std_bf(std_bf),
	_s(n_sample),
	_alpha(time_mod),
	_dims(dims)
{
	assert(_mean_w.size() == _cov_w.cols());
	assert(_cov_w.rows() == _cov_w.cols());
	assert(w.size() % dims == 0);

	_phase = compute_phase();
	_phi = generate_basis_function(_phase);
}

Eigen::VectorXd ProMP::compute_phase(size_t timesteps) const
{
	return Eigen::VectorXd::LinSpaced(timesteps, 0.0, 1.0);
}

Eigen::VectorXd ProMP::compute_phase()
{
	return compute_phase(static_cast<size_t>(_s / _alpha));
}

Eigen::VectorXd ProMP::get_phase_from_time(int time_pos) const
{
	Eigen::VectorXd phase(1);
	phase(0) = static_cast<double>(time_pos) / (static_cast<int>(_s / _alpha) - 1 );
	return phase;
}

const Eigen::VectorXd& ProMP::get_phase() const
{
	return _phase;
}

void ProMP::set_phase(const Eigen::VectorXd& z)
{
	_phase = z;
	_phi = generate_basis_function(_phase);
}


Eigen::MatrixXd ProMP::generate_basis_function(const Eigen::VectorXd& phase) const
{
	// create vector of rbf centers
	Eigen::VectorXd centers = Eigen::VectorXd::LinSpaced(_num_bf, 0.0 - 2 * _std_bf , 1.0 + 2 * _std_bf);
	// create a matrix with n_rbf rows phase.size() cols. each row is the phase
	Eigen::MatrixXd phase_bf_mtx = phase.replicate(1, _num_bf).transpose();	
	//create a matrix with n_rbf rows and phase.size() cols. each col is centers
	Eigen::MatrixXd c_mtx = centers.replicate(1, phase.rows());
	// compute t-c(i)
	Eigen::MatrixXd phase_diff = phase_bf_mtx - c_mtx;
	// compute rbf matrix and normalized it colwise
	Eigen::MatrixXd phi = rbf_gaussian(phase_diff);
	Eigen::VectorXd sum_bf = phi.colwise().sum();
	for (int i=0; i < phase.rows(); ++i)
		phi.col(i) = phi.col(i) / sum_bf(i);

	return phi;
}

Eigen::MatrixXd ProMP::repeat_block_diagonal(const Eigen::MatrixXd& mtx, size_t t) const
{
	size_t r = mtx.rows();
	size_t c = mtx.cols();
	Eigen::MatrixXd extended = Eigen::MatrixXd::Zero(t*r, t*c);
	for(int i = 0; i < t; ++i)
		extended.block(i*r, i*c, r, c) = mtx;
	return extended;
}

const Eigen::MatrixXd& ProMP::get_basis_function() const
{
	return _phi;
}

const Eigen::VectorXd& ProMP::get_weights() const
{
	return _mean_w;
}

const Eigen::MatrixXd& ProMP::get_covariance() const
{
	return _cov_w;
}

double ProMP::get_std_bf() const
{
	return _std_bf;
}

size_t ProMP::get_dims() const
{
	return _dims;
}

int ProMP::get_n_samples() const
{
	return _s;
}

int ProMP::get_traj_length() const
{
	int traj_length = static_cast<int>(_s / _alpha);
	return traj_length;
}

double ProMP::get_time_mod() const
{
	return _alpha;
}

void ProMP::set_ridge_factor(double ridge_factor)
{
	_ridge_factor = ridge_factor;
}

Eigen::MatrixXd ProMP::compute_ridge_pseudo_inverse(const Eigen::MatrixXd& m)
{
	auto eye = Eigen::MatrixXd::Identity(m.cols(), m.cols());
	return (m.transpose() * m + _ridge_factor * eye).inverse() * m.transpose();
}

void ProMP::train(const std::vector<Trajectory>& database)
{
	size_t timesteps = _s;
	
	Eigen::MatrixXd phi_t_inv = compute_ridge_pseudo_inverse(_phi.transpose());

	// stack demonstrations weights matrices into a single matrix
	// linear ridge regression for each demonstration
	// cols alternates different dimension for all motions. ex [w1_d1, w1_d2, w2_d1, w2_d2 ..., wn_d1, wn_d2] (2d example)
	Eigen::MatrixXd w_mtx(_num_bf, _dims * _num_demos);
	for(int d=0; d < database.size(); ++d)
	{
		assert(database[d].timesteps() == _s);
		assert(database[d].dims() == _dims);
		w_mtx.block(0, d * _dims, _num_bf, _dims) = phi_t_inv * database[d].matrix();
	}

	// view demonstrations' weight matrices as vectors
	auto w_mtx_as_vec = Eigen::Map<Eigen::MatrixXd>(w_mtx.data(), _num_bf * _dims, _num_demos);
	
	_mean_w = w_mtx_as_vec.rowwise().mean();

	if(_num_demos == 1)
	{
		_cov_w = 1e-3 * Eigen::MatrixXd::Identity(_num_bf, _num_bf);
	}
	else
	{
		Eigen::MatrixXd centered = (w_mtx_as_vec.colwise() - w_mtx_as_vec.rowwise().mean()).transpose();
		_cov_w = (centered.adjoint() * centered) / static_cast<double>(_num_demos - 1);
	}
}

void ProMP::condition_start(const Eigen::VectorXd& start, const Eigen::MatrixXd& std)
{
	condition_via_point(0.0, start, std);
}

void ProMP::condition_goal(const Eigen::VectorXd& goal ,const Eigen::MatrixXd& std)
{
	condition_via_point(1.0, goal, std);	
}

Eigen::MatrixXd ProMP::generate_trajectory(size_t req_num_steps) const
{
	//modulate speed
	auto desired_phase = compute_phase(req_num_steps);
	// generate basis function for new phase parameterization
	auto phi = generate_basis_function(desired_phase);
	//compute trajectory from weights and new phase
	auto traj = phi.transpose() * Eigen::Map<const Eigen::MatrixXd>(_mean_w.data(), _num_bf, _dims);

	return traj;
}

Eigen::MatrixXd ProMP::generate_trajectory_with_speed(double req_phase_speed) const
{
	return generate_trajectory(_s * _alpha / req_phase_speed); //phase_speed_from_steps(req_num_steps));
}

Eigen::MatrixXd ProMP::generate_trajectory() const
{
	//compute trajectory from weights and basis function
	return _phi.transpose() *  Eigen::Map<const Eigen::MatrixXd>(_mean_w.data(), _num_bf, _dims);
}

Eigen::MatrixXd ProMP::generate_trajectory_at(const Eigen::VectorXd& phase) const
{
	auto phi = generate_basis_function(phase);
	auto traj = phi.transpose() * Eigen::Map<const Eigen::MatrixXd>(_mean_w.data(), _num_bf, _dims);
	return traj;
}

Eigen::MatrixXd ProMP::gen_traj_std_dev(size_t req_num_steps) const
{
	Eigen::MatrixXd phi;

	if(req_num_steps == 0)
	{
		phi = _phi;
		req_num_steps = _s;
	}
	else
	{
		auto desired_phase = compute_phase(req_num_steps);	// calculate new phase object for required steps
		phi = generate_basis_function(desired_phase); // generate basis function for new phase parameterization
	}
	
	Eigen::MatrixXd std_dev_matrix(req_num_steps, _dims);
	for(int i = 0; i < _dims; ++i)
	{
		// for each dof i, compute diagonal of (phi' * cov_w_i) * phi
		Eigen::MatrixXd phi_t_covw = phi.transpose() * _cov_w.block(i * _num_bf, i * _num_bf, _num_bf, _num_bf);
		for(int j=0; j < req_num_steps; ++j)
			std_dev_matrix(j, i) = phi_t_covw.row(j) * phi.col(j);
		std_dev_matrix.col(i) = std_dev_matrix.col(i).cwiseSqrt();
	}

	return std_dev_matrix;

	// method equivalent to: (extended_phi is repeat_block_diagonal(phi))
	// Eigen::MatrixXd traj_std = extended_phi.transpose() * _cov_w * extended_phi;
	// traj_std = traj_std.cwiseSqrt();
	// Eigen::VectorXd diagonal = traj_std.diagonal(); // store diagonal in non temp object to access data()
	//return Eigen::Map<Eigen::MatrixXd>(diagonal.data(), req_num_steps, _dims);
}

std::vector<Eigen::MatrixXd> ProMP::generate_trajectory_covariance(size_t req_num_steps) const
{
	Eigen::MatrixXd phi;

	if(req_num_steps == 0)
	{
		phi = _phi;
		req_num_steps = _s;
	}
	else
	{
		auto desired_phase = compute_phase(req_num_steps);	//calculate phase speed and new phase object for required steps
		phi = generate_basis_function(desired_phase); // generate basis function for new phase parameterization
	}

	// computing extended_phi.transpose * cov(w).  (avoid computation with zero sub matrices)
	Eigen::MatrixXd phi_t_w(phi.cols() * _dims, _cov_w.cols());
	for(int i=0; i < _dims; ++i)
		for(int j=0; j < _dims; ++j)
			phi_t_w.block(req_num_steps * i, _num_bf * j, req_num_steps, _num_bf) = phi.transpose() * _cov_w.block(_num_bf * i, _num_bf * j, _num_bf, _num_bf);

	/// equivalent to (a little slower)
	// RepeatBlockDiagonalMatrix extended_phi(phi, _dims);
	// Eigen::MatrixXd phi_t_w = extended_phi.transpose() * _cov_w;

	// equivalent to (MUCH SLOWER)
	// Eigen::MatrixXd extended_phi = repeat_block_diagonal(phi, _dims);
	// Eigen::MatrixXd phi_t_w = e_phi.transpose() * _cov_w;

	// computing essential data of extended_phi.transpose * cov(w) * extended_phi
	std::vector<Eigen::MatrixXd> trajectory_covariance(req_num_steps);
	for(int t=0; t < req_num_steps; ++t)
	{
		Eigen::MatrixXd step_covariance(_dims, _dims);

		for(int i = 0; i < _dims; ++i) // compare all sample of dimension i
			for(int j = i; j < _dims; ++j) // with all samples of dimension j (but just to have upper triangular, then copy)
				step_covariance(i, j) = phi_t_w.row(i * _s + t).segment(_num_bf * j, _num_bf) * phi.col(t);

		step_covariance.triangularView<Eigen::Lower>() = step_covariance.transpose().triangularView<Eigen::Lower>();
		trajectory_covariance[t] = step_covariance;
	}

	return trajectory_covariance;
}

void ProMP::condition_via_point(int time, const Eigen::VectorXd& via_point, const Eigen::MatrixXd& std)
{
	condition_via_points({std::make_tuple<>(time, via_point, std)});
}

void ProMP::condition_via_points(const std::vector<std::tuple<int, Eigen::VectorXd, Eigen::MatrixXd>>& via_points)
{
	for(const auto& via_point : via_points)
	{
		auto phase_obs = get_phase_from_time(std::get<0>(via_point));

		const Eigen::VectorXd& obs = std::get<1>(via_point);
		const Eigen::MatrixXd& sig_obs = std::get<2>(via_point);

		assert(obs.size() == _dims);
		assert(sig_obs.rows() == sig_obs.cols() && sig_obs.cols() == _dims);
		
		// equivalent to
		// auto phi_obs = repeat_block_diagonal(generate_basis_function(phase_obs), _dims);  // TODO optimize (this matrix grow a lot)

		RepeatBlockDiagonalMatrix phi_obs(generate_basis_function(phase_obs), _dims);
		//Eigen::MatrixXd ridge = _ridge_factor * Eigen::MatrixXd::Identity(_dims, _dims); // might be useful with a lot of points ????
		Eigen::MatrixXd L = _cov_w * phi_obs * (sig_obs + phi_obs.transpose() * _cov_w * phi_obs).inverse(); // NOT use auto, broken with RepeatBlockDiagonalMatrix

		_mean_w = _mean_w + L * (obs - phi_obs.transpose() * _mean_w);
		_cov_w = _cov_w - L * phi_obs.transpose() * _cov_w;
	}
}

Eigen::MatrixXd ProMP::rbf_gaussian(Eigen::MatrixXd& phase_diff) const
{
	Eigen::MatrixXd phase_diff_F(phase_diff.rows(), phase_diff.cols());
	for(int i=0; i < phase_diff.rows(); ++i)
		for(int j=0; j < phase_diff.cols(); ++j)
			phase_diff_F(i,j) = std::exp(-0.5 * pow(phase_diff(i,j), 2) / _std_bf);

	return phase_diff_F;
}

void ProMP::compute_delta(double K)
{
	auto eye = Eigen::MatrixXd::Identity(_num_bf, _num_bf);
	// TODO COMPUTE pinv just in train method and keep it as attribute
	Eigen::MatrixXd phi_t_inv = compute_ridge_pseudo_inverse(_phi.transpose());
	_delta_wy = K * phi_t_inv * gen_traj_std_dev();

	// return Eigen::Map<Eigen::VectorXd>(_delta_wy.data(), _mean_w.size());
}

Eigen::VectorXd ProMP::get_upper_weights(double K)
{
	compute_delta(K);
	return _mean_w + Eigen::Map<Eigen::VectorXd>(_delta_wy.data(), _mean_w.size());

}

Eigen::VectorXd ProMP::get_lower_weights(double K)
{
	compute_delta(K);
	return _mean_w - Eigen::Map<Eigen::VectorXd>(_delta_wy.data(), _mean_w.size());	
}

std::ostream& operator<<(std::ostream &out, const ProMP &mp)
{
	out << "The weights of are: ";

	auto weights = mp.get_weights();
	for(int i=0; i< weights.size(); i++){
		out << weights[i] << ", ";
	}

	return out;
}

} // end namespace promp