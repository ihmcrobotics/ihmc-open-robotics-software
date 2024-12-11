#include "constant-position-controller.hpp"

#include "types.hpp"

namespace ihmc
{
    ConstantPositionController::ConstantPositionController(const double default_stiffness, const double default_damping, const int number_of_joints)
    {
        number_of_joints_ = number_of_joints;
        desired_joint_velocities_ = Eigen::VectorXd(number_of_joints_);
        desired_joint_torques_ = Eigen::VectorXd(number_of_joints_);
        desired_joint_stiffnesses_ = Eigen::VectorXd(number_of_joints_);
        desired_joint_damping_ = Eigen::VectorXd(number_of_joints_);

        desired_joint_velocities_.setZero();
        desired_joint_torques_.setZero();
        desired_joint_stiffnesses_.fill(default_stiffness);
        desired_joint_damping_.fill(default_damping);
    }

    bool ConstantPositionController::setHomeJointConfiguration(const double* configuration_data, int configuration_rows)
    {
        if (configuration_rows != number_of_joints_)
            return false;

        const VectorViewReadOnly configuration(configuration_data, configuration_rows);
        home_configuration_ = configuration;

        return true;
    }

    void compute(const Eigen::Vector3d& base_position, const Eigen::Vector4d& base_orientation, const Eigen::VectorXd& joint_positions,
                 const Eigen::Vector3d& base_linear_velocity, const Eigen::Vector3d& base_angular_velocity, const Eigen::VectorXd& joint_velocities)
    {
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