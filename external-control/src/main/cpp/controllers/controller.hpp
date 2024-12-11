#pragma once

namespace ihmc
{
    class Controller
    {
    public:
        virtual void compute(const Eigen::Vector3d& base_position, const Eigen::Vector4d& base_orientation, const Eigen::VectorXd& joint_positions,
                             const Eigen::Vector3d& base_linear_velocity, const Eigen::Vector3d& base_angular_velocity, const Eigen::VectorXd& joint_velocities) = 0;


        virtual Eigen::VectorXd get_desired_joint_positions() const;

        virtual Eigen::VectorXd get_desired_joint_velocities() const;

        virtual Eigen::VectorXd get_desired_joint_torques() const;

        virtual Eigen::VectorXd get_desired_joint_stiffnesses() const;

        virtual Eigen::VectorXd get_desired_joint_damping() const;

    };
}