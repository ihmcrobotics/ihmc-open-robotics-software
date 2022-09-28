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

#ifndef PROMP_HPP
#define PROMP_HPP

#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <promp/trajectory_group.hpp>

namespace promp {

    /**
     * @brief Class that implements a multi-dimensional Probabilistic Motion Primitive. References:
     * - Paraschos A, Daniel C, Peters J, Neumann G. Probabilistic movement primitives. Advances in neural information processing systems. 2013. <a href="https://www.ias.informatik.tu-darmstadt.de/uploads/Publications/Paraschos_NIPS_2013.pdf">[pdf]</a>
	 * - Paraschos A, Daniel C, Peters J, Neumann G. Using probabilistic movement primitives in robotics. Autonomous Robots. 2018 Mar;42(3):529-51. <a href="https://www.ias.informatik.tu-darmstadt.de/uploads/Team/AlexandrosParaschos/promps_auro.pdf">[pdf]</a>.
     */
    class ProMP {
    public:
        /** \brief constructor: The constructor will parameterize a phase vector, and compute the basis function matrix
         * \f$ \Psi \f$ for all phase steps. Then, for each demonstration within the std::vector \p data, it will estimate a.
         * vector of basis functions' weights \f$ w_i \f$.
         * Lastly, it fits a gaussian over all the weight vectors to obtain \f$ \mu_w \f$, and \f$ \Sigma_w \f$
         *
         *	\param data 	 vector of trajectories. All the trajectories MUST have the same length and same dimension.
         *	\param num_bf 	 number of basis functions
         *	\param std_bf	 standard deviation; this is set automatically to  1.0 / (n_rbf*n_rbf) if std_bf <= 0
         */
        ProMP(const std::vector<Trajectory>& data, int num_bf, double std_bf = -1)
            : _num_bf(num_bf),
              _alpha(1.0)
        {
            init(data, std_bf);
        }

        /** \brief constructor: The constructor will parameterize a phase vector, and compute the basis function matrix
         * \f$ \Psi \f$ for all phase steps. Then, for each demonstration within the std::vector \p data, it will estimate a.
         * vector of basis functions' weights \f$ w_i \f$.
         * Lastly, it fits a gaussian over all the weight vectors to obtain \f$ \mu_w \f$, and \f$ \Sigma_w \f$
         *
         *	\param data 	 vector of trajectories. All the trajectories MUST have the same length and same dimension.
         *	\param num_bf 	 number of basis functions
         *	\param std_bf	 standard deviation; this is set automatically to  1.0 / (n_rbf*n_rbf) if std_bf <= 0
         */
        ProMP(const TrajectoryGroup& data, int num_bf, double std_bf = -1)
            : _num_bf(num_bf),
              _alpha(1.0)
        {
            init(data.trajectories(), std_bf);
        }

        /**
         * @brief      This is an alternate constructor that uses prelearned weights, covariance and number of samples in the trajectory
         *
         * @param[in]  w    	The mean of weights' distribution
         * @param[in]  cov_w    The co-variance of weights' distribution
         * @param[in]  std_bf   The standard deviation of each basis function
         * @param[in]  n_sample The number of samples required for the trajectory
         */
        ProMP(const Eigen::VectorXd& w, const Eigen::MatrixXd& cov_w, double std_bf, int n_sample, size_t dims, double time_mod = 1.0);

        /**
         * @brief      Gets the basis function matrix for the current trained ProMP.
         *
         * @return     The basis function \f$ \Psi \f$.
         */
        const Eigen::MatrixXd& get_basis_function() const;

        /** \brief	generates basis functions
         *	\param phase
         *	\return matrix of basis functions
         */
        Eigen::MatrixXd generate_basis_function(const Eigen::VectorXd& phase) const;

        /** \brief	maps time vector into a phase vector give a desired number of timesteps
         */
        Eigen::VectorXd compute_phase(size_t timesteps) const;

        /**
         * @brief      Gets the phase vector.
         *
         * @return     The phase vector.
         */
        const Eigen::VectorXd& get_phase() const;

        /**
         * @brief      Gets the vector of the mean of all weights \f$ \mu_w \f$.
         *
         * @return     The weights.
         */
        const Eigen::VectorXd& get_weights() const;

        /**
         * @brief      Gets the co-variance matrix \f$ \Sigma_w \f$.
         *
         * @return     The co-variance matrix.
         */
        const Eigen::MatrixXd& get_covariance() const;

        /**
         * @brief      Gets the number of samples for the trajectory \f$ \s_ \f$.
         *
         * @return     The number of samples for the trajectory
         */
        int get_n_samples() const;

        /**
         * @brief      Gets the trajectory length.
         *
         * @return     The trajectory length
         */
        int get_traj_length() const;

        /**
         * @brief      Gets the std deviation \f$ \std_bf \f$ of the ProMP
         * @return     The std deviation.
         */
        double get_std_bf() const;

        /**
         * @brief	return the number of dimensions represented and generated by the promp
         * @return	Number of dimensions of the generated function
         */
        size_t get_dims() const;

        /**
         * @brief      Gets the average time modulation in the demonstrations used to train the ProMP
         * @return     m_alpha_ The avergae demo time modulation
         */
        double get_mean_demo_time_mod() const;

        /**
         * @brief		Set the ridge factor value which condition the pphi inverse. Helps against singularities
         * @param[in]	ridge_factor
         */
        void set_ridge_factor(double ridge_factor);

        /**
         * @brief      Generates MEAN trajectory based on current weights distribution and rbf
         *
         * @return     Mean trajectory
         */
        Eigen::MatrixXd generate_trajectory() const;

        /**
         * @brief      Generates MEAN trajectory based on current weights distribution and
         * a required number of steps
         *
         * @param[in]  req_num_steps  The requested number of steps for trajectory
         *
         * @return     Mean trajectory
         */
        Eigen::MatrixXd generate_trajectory(size_t req_num_steps) const;

        /**
         * @brief      Generates MEAN trajectory based on current weights distributions
         * and a required phase speed \f$ \dot{z}_t \f$.
         *
         * @param[in]  req_phase_speed  The request phase speed. <b> To play trajectory at orignal speed set
         * \f$ \dot{z}_t = 1.0 \f$ </b>.
         *
         * @return     { description_of_the_return_value }
         */
        Eigen::MatrixXd generate_trajectory_with_speed(double req_phase_speed) const;

        Eigen::MatrixXd generate_trajectory_at(const Eigen::VectorXd& phase) const;

        /** \brief	set desired goal/end point for trajectory
         *	\param 	goal 	desired value at end
         *	\param	std 	desired standard deviation. <b> typically around \f$ 10^{-6}\f$ for accuracy </b>
         *	\return
         */
        void condition_goal(const Eigen::VectorXd& goal, const Eigen::MatrixXd& std);

        /** \brief	set desired start/initial point for trajectory
         *	\param 	start 	desired value at start
         *	\param	std   	desired standard deviation.<b> typically around \f$ 10^{-6}\f$ for accuracy </b>
         *	\return
         */
        void condition_start(const Eigen::VectorXd& start, const Eigen::MatrixXd& std);

        /**
         * @brief      Generates standard deviation vector with the standard deviation for every time step, with a certain number of time steps
         *
         * @param[in]  req_num_steps  The requested number steps. if <= 0 (default) use the internal phase parametrization
         *
         * @return     Standard Deviation Vector \f$ DIAG( \Sigma ) \f$
         */
        Eigen::MatrixXd gen_traj_std_dev(size_t req_num_steps = 0) const;

        /**
         * @brief      Generates step covariance matrices for for each step of the trajectory, with a certain number of time steps
         *
         * @param[in]  req_num_steps  The requested number steps. if <= 0 (default) use the internal phase parametrization
         *
         * @return     std::vector containg and entry for each step covariance matrix
         */
        std::vector<Eigen::MatrixXd> generate_trajectory_covariance(size_t req_num_steps = 0) const;

        /**
         * @brief      Conditions all via points registered in 'viaPoints_'.
         * It updates  \a _mean_w and _cov_w  and clear _via_points vector
         */
        void condition_via_points(const std::vector<std::tuple<int, Eigen::VectorXd, Eigen::MatrixXd>>& via_points);

        /**
         * @brief      Conditions all via points registered in 'viaPoints_'.
         * It updates  \a _mean_w and _cov_w  and clear _via_points vector
         */
        /** \brief	set via point for trajectory
         *	\param 	t 			time at which via point is to be added (between 0 and LAST TIME STEP)
         *	\param	via_point 	desired value at via point
         *	\param	std   		desired standard deviation.<b> typically around \f$ 10^{-6}\f$ for accuracy </b>
         *	\return
         *	\todo Use phase \f$ z_t\f$ instead of time step t.
         */
        void condition_via_point(int t, const Eigen::VectorXd& via_point, const Eigen::MatrixXd& std);

        Eigen::VectorXd get_upper_weights(double K);

        Eigen::VectorXd get_lower_weights(double K);

        /**
         * @brief      Overloads << operator in order to print the weights of a given promp
         *
         * @param      out   The out
         * @param[in]  mp    THIS promp
         *
         */
        friend std::ostream& operator<<(std::ostream& out, const ProMP& mp);

        /**
         * @brief generate the speed from a number of steps
         * @param steps number of steps required in the trajectory
         * @return double the phase speed to obtain the desired number of steps in the trajectory
         */
        inline double phase_speed_from_steps(int steps) { return static_cast<double>(_s) / steps; }

    private:
        /** \brief	maps time vector into a phase vector using the inner timesteps parametrization
         */
        Eigen::VectorXd compute_phase();

        /**
         * @brief      train using demonstration trajectories to obtain average weights
         *
         * @param      database  matrix of demonstration trajectories
         */
        void train(const std::vector<Trajectory>& database);

        /**
         * @brief      extend matrix mtx into a block diagonal matrix, repeating it t times
         *
         *
         */
        Eigen::MatrixXd repeat_block_diagonal(const Eigen::MatrixXd& mtx, size_t t) const;

        /**
         * @brief      compute the ridge pseudo inverse of matrix m
         *
         * @param      m  matrix to be inverted
         */
        Eigen::MatrixXd compute_ridge_pseudo_inverse(const Eigen::MatrixXd& m);

        /** \brief 				get phase for a specific point in the trajectory
         *	\param				time step of which phase is to be found
         *	\return 			phase
         */
        Eigen::VectorXd get_phase_from_time(int time_steps) const;

        /** \brief	maps time vector into a phase vector.
         */
        void set_phase(const Eigen::VectorXd& z);

        /**
         * @brief      Calculates the phase for a required number of steps
         *
         * @param[in]  req_num_steps  The required number of steps
         */
        void compute_phase_for_steps(int req_num_steps);

        /**
         * @brief      Gets the time modulation.
         *
         * @return     The time modulation
         */
        double get_time_mod() const;

        // called by the constructors to init everything
        void init(const std::vector<Trajectory>& data, double std_bf);

        /// utility function
        Eigen::MatrixXd rbf_gaussian(Eigen::MatrixXd& phase_diff) const;

        void compute_delta(double K);

        //! ProMp identifier
        std::string _id;
        //! number of basis functions
        int _num_bf;
        //! standard deviation of basis functions
        double _std_bf;
        //! number of data dimensions
        size_t _dims;

        //! Basis Function Matrix
        Eigen::MatrixXd _phi;
        Eigen::MatrixXd _phi_inv;
        Eigen::MatrixXd _phase_diff;

        //! number of demonstrations
        int _num_demos;
        //! number of points in the entire trajectory
        int _traj_length;

        //! average trajectory directly from demonstrations
        Eigen::VectorXd _means;
        //! average time modulation in the demonstrations used to train the ProMP
        double _mean_alpha;

        //! phase vector
        Eigen::VectorXd _phase;
        //! time modulation
        double _alpha = 1.0;
        //! ridge factor
        double _ridge_factor = 1e-10;
        //! number of samples used to rescale all the trajectories to same duration
        int _s;
        //! Mean of weights' distribution
        Eigen::VectorXd _mean_w;
        //! Co-variance of weights' distribution
        Eigen::MatrixXd _cov_w;

        Eigen::MatrixXd _delta_wy;
    };

    /**
     * Shared Pointer Declaration For ProMP Class
     */
    typedef std::shared_ptr<ProMP> ProMpPtr;

} // end namespace promp

#endif // PROMP_HPP