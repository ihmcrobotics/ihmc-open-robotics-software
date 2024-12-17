#include "controller.hpp"

namespace ihmc
{
    void Controller::compute(const Eigen::Vector3d& base_position, const Eigen::Vector4d& base_orientation, const Eigen::VectorXd& joint_positions,
                                                          const Eigen::Vector3d& base_linear_velocity, const Eigen::Vector3d& base_angular_velocity, const Eigen::VectorXd& joint_velocities)
    {
    }

    Eigen::VectorXd Controller::get_desired_joint_positions()
    {
    }

    Eigen::VectorXd Controller::get_desired_joint_velocities()
    {
    }

    Eigen::VectorXd Controller::get_desired_joint_torques()
    {
    }

    Eigen::VectorXd Controller::get_desired_joint_stiffnesses()
    {
    }

    Eigen::VectorXd Controller::get_desired_joint_damping()
    {
    }
}
