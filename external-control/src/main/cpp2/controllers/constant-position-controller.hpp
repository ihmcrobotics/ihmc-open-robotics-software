#pragma once

#include <Eigen/Core>

#include "controller.hpp"

namespace ihmc
{
    class ConstantPositionController final : public Controller
    {
    public:

        ConstantPositionController() = default;

        explicit ConstantPositionController(const double default_stiffness, const double default_damping, const int number_of_joints);

        virtual ~ConstantPositionController() = default;

        bool setHomeJointConfiguration(const double* configuration_data, int rows);

        void compute(const Eigen::Vector3d& base_position, const Eigen::Vector4d& base_orientation, const Eigen::VectorXd& joint_positions,
                                     const Eigen::Vector3d& base_linear_velocity, const Eigen::Vector3d& base_angular_velocity, const Eigen::VectorXd& joint_velocities) override;

        Eigen::VectorXd get_desired_joint_positions() const override;

        Eigen::VectorXd get_desired_joint_velocities() const override;

        Eigen::VectorXd get_desired_joint_torques() const override;

        Eigen::VectorXd get_desired_joint_stiffnesses() const override;

        Eigen::VectorXd get_desired_joint_damping() const override;

    private:
        int number_of_joints_;

        Eigen::VectorXd home_configuration_;
        Eigen::VectorXd desired_joint_velocities_;
        Eigen::VectorXd desired_joint_torques_;
        Eigen::VectorXd desired_joint_stiffnesses_;
        Eigen::VectorXd desired_joint_damping_;
    };
}