#pragma once

#include "types.hpp"
#include "zmq.hpp"

namespace ihmc
{
    class ZMQController
    {
    public:

        explicit ZMQController(const double default_stiffness, const double default_damping, const int number_of_joints);

        virtual ~ZMQController() = default;

        void startSocket();

        void stopSocket();

        void resize(const double default_stiffness, const double default_damping, const int number_of_joints);

        bool setHomeJointConfiguration(const double* configuration_data, int rows);

        void compute(const double current_time,
                    const double* x_data, int x_rows,
                    const double* u_data, int u_rows,
                    int behavior_status);

        Eigen::VectorXd get_desired_joint_positions() const;

        Eigen::VectorXd get_desired_joint_velocities() const;

        Eigen::VectorXd get_desired_joint_torques() const;

        Eigen::VectorXd get_desired_joint_stiffnesses() const;

        Eigen::VectorXd get_desired_joint_damping() const;

        int get_debug_data_size();

        Eigen::VectorXd get_debug_data() const;

    private:
        int number_of_joints_;
        const int debug_data_size_ = 1 + 4 + 6;

        Eigen::VectorXd home_configuration_;
        Eigen::VectorXd desired_joint_velocities_;
        Eigen::VectorXd desired_joint_torques_;
        Eigen::VectorXd desired_joint_stiffnesses_;
        Eigen::VectorXd desired_joint_damping_;
        Eigen::VectorXd debug_data_;

        // zmq
        zmq::context_t context;
        zmq::socket_t socket_cpp;
        zmq::pollitem_t items[1];


    };
}