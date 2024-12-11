#include "external-control.hpp"

#include <iostream>
#include "types.hpp"

namespace ihmc
{
    ExternalControlImpl::ExternalControlImpl(const double default_position, const double default_damping, const int number_of_joints)
    {
        constant_position_controller_ = ConstantPositionController(default_position, default_damping, number_of_joints);
        desired_state_data_ = Eigen::VectorXd(13 + 2 * number_of_joints);
        desired_control_data_ = Eigen::VectorXd(number_of_joints);
        p_gains_ = Eigen::VectorXd(number_of_joints);
        d_gains_ = Eigen::VectorXd(number_of_joints);
        number_of_joints_ = number_of_joints;
    }



    bool ExternalControlImpl::updateRobotState(const double current_time,
                                               const double* x_data, int x_rows,
                                               const double* u_data, int u_rows,
                                               const bool left_in_contact, const bool right_in_contact,
                                               const double* foot_locations, int foot_locations_rows,
                                               const int hardware_status)
    {
        if (hardware_status == 0)
        {
            const VectorViewReadOnly state(x_data, x_rows);
            const Eigen::Vector3d base_position = state.head(3);
            const Eigen::Vector4d base_orientation = state.segment(3, 4);
            const Eigen::VectorXd joint_positions = state.segment(7, number_of_joints_);
            const Eigen::VectorXd base_linear_velocity = state.segment(7 + number_of_joints_, 3);
            const Eigen::VectorXd base_angular_velocity = state.segment(10 + number_of_joints_, 3);
            const Eigen::VectorXd joint_velocities = state.tail(number_of_joints_);

            constant_position_controller_.compute(base_position,
                                                  base_orientation,
                                                  joint_positions,
                                                  base_linear_velocity,
                                                  base_angular_velocity,
                                                  joint_velocities);

            desired_state_data_.head(7) << base_position, base_orientation;
            desired_state_data_.segment(7, number_of_joints_) = constant_position_controller_.get_desired_joint_positions();
            desired_state_data_.segment(7 + number_of_joints_, 6) << base_linear_velocity, base_angular_velocity;
            desired_state_data_.tail(number_of_joints_) = constant_position_controller_.get_desired_joint_velocities();

            desired_control_data_ = constant_position_controller_.get_desired_joint_torques();
            p_gains_ = constant_position_controller_.get_desired_joint_stiffnesses();
            d_gains_ = constant_position_controller_.get_desired_joint_damping();

            return true;
        }
        else
        {
            std::cout << "There is no control mode defined for " << hardware_status << std::endl;
            return false;
        }
    }

    bool ExternalControlImpl::getSolution(double* state_data_to_pack, int state_rows,
                         double* control_data_to_pack, int control_rows,
                         double* p_gains_to_pack, int p_gain_rows,
                         double* d_gains_to_pack, int d_gain_rows) const
    {
        VectorView state_data(state_data_to_pack, state_rows);
        VectorView control_data(control_data_to_pack, control_rows);
        VectorView p_gains(p_gains_to_pack, p_gain_rows);
        VectorView d_gains(d_gains_to_pack, d_gain_rows);

        state_data = desired_state_data_;
        control_data = desired_control_data_;
        p_gains = p_gains_;
        d_gains = d_gains_;

        return true;
    }

    bool ExternalControlImpl::setHomeJointConfiguration(const double* configuration_data, int rows)
    {
        return constant_position_controller_.setHomeJointConfiguration(configuration_data, rows);
    }
}
