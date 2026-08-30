#pragma once

#include "types.hpp"

namespace ihmc
{
    class ConstantPositionController
    {
    public:

        ConstantPositionController() = default;

        explicit ConstantPositionController(const double default_stiffness, const double default_damping, const int number_of_joints);

        virtual ~ConstantPositionController() = default;

        void resize(const double default_stiffness, const double default_damping, const int number_of_joints);

        bool setHomeJointConfiguration(const double* configuration_data, int rows);

        Eigen::VectorXd get_desired_joint_positions() const;

        Eigen::VectorXd get_desired_joint_velocities() const;

        Eigen::VectorXd get_desired_joint_torques() const;

        Eigen::VectorXd get_desired_joint_stiffnesses() const;

        Eigen::VectorXd get_desired_joint_damping() const;

    private:
        int number_of_joints_;

        Eigen::VectorXd home_configuration_;
        Eigen::VectorXd desired_joint_velocities_;
        Eigen::VectorXd desired_joint_torques_;
        Eigen::VectorXd desired_joint_stiffnesses_;
        Eigen::VectorXd desired_joint_damping_;
    };
}