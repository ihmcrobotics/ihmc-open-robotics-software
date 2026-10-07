#include "constant-position-controller.hpp"

#include <iostream>

namespace ihmc
{
    ConstantPositionController::ConstantPositionController(const double default_stiffness, const double default_damping, const int number_of_joints)
    {
        number_of_joints_ = number_of_joints;
        desired_joint_velocities_.resize(number_of_joints_);
        desired_joint_torques_.resize(number_of_joints_);
        desired_joint_stiffnesses_.resize(number_of_joints_);
        desired_joint_damping_.resize(number_of_joints_);

        desired_joint_velocities_.setZero();
        desired_joint_torques_.setZero();
        desired_joint_stiffnesses_.fill(default_stiffness);
        desired_joint_damping_.fill(default_damping);
    }

    void ConstantPositionController::resize(const double default_stiffness, const double default_damping, const int number_of_joints)
    {
        std::cout << "resizing constant position controller" << std::endl;
        number_of_joints_ = number_of_joints;
        desired_joint_velocities_.resize(number_of_joints_);
        desired_joint_torques_.resize(number_of_joints_);
        desired_joint_stiffnesses_.resize(number_of_joints_);
        desired_joint_damping_.resize(number_of_joints_);

        desired_joint_velocities_.setZero();
        desired_joint_torques_.setZero();
        desired_joint_stiffnesses_.fill(default_stiffness);
        desired_joint_damping_.fill(default_damping);
    }

    bool ConstantPositionController::setHomeJointConfiguration(const double* configuration_data, int configuration_rows)
    {
        if (configuration_rows != number_of_joints_)
        {
            std::cout << "The size of the home configuration is incorrect" << std::endl;
            return false;
        }

        const VectorViewReadOnly configuration(configuration_data, configuration_rows);
        home_configuration_ = configuration;

        std::cout << "home configuration is set" << std::endl;

        return true;
    }

    Eigen::VectorXd ConstantPositionController::get_desired_joint_positions() const
    {
        return home_configuration_;
    }

    Eigen::VectorXd ConstantPositionController::get_desired_joint_velocities() const
    {
        return desired_joint_velocities_;
    }

    Eigen::VectorXd ConstantPositionController::get_desired_joint_torques() const
    {
        return desired_joint_torques_;
    }

    Eigen::VectorXd ConstantPositionController::get_desired_joint_stiffnesses() const
    {
        return desired_joint_stiffnesses_;
    }

    Eigen::VectorXd ConstantPositionController::get_desired_joint_damping() const
    {
        return desired_joint_damping_;
    }
}