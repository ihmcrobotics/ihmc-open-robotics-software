#include "external-control.hpp"

#include <iostream>
#include "types.hpp"

namespace ihmc
{
    ExternalControlImpl::ExternalControlImpl(const double default_stiffness, const double default_damping, const int number_of_joints)
        : zmq_controller_(default_stiffness, default_damping, number_of_joints)
    {
       std::cout << "Creating external control object." << std::endl;

        constant_position_controller_.resize(default_stiffness, default_damping, number_of_joints);
        desired_state_data_.resize(13 + 2 * number_of_joints);
        desired_control_data_.resize(number_of_joints);
        p_gains_.resize(number_of_joints);
        d_gains_.resize(number_of_joints);
        number_of_joints_ = number_of_joints;

        std::cout << "Created external control object." << std::endl;
    }



    bool ExternalControlImpl::updateRobotState(const double current_time,
                                               const double* x_data, int x_rows,
                                               const double* u_data, int u_rows,
                                               const bool left_in_contact, const bool right_in_contact,
                                               const double* foot_locations, int foot_locations_rows,
                                               const int hardware_status, const int behavior_status)
    {
        if (x_rows != 13 + 2 * number_of_joints_)
        {
            std::cout << "The size of the state vector is incorrect" << std::endl;
            return false;
        }
        if (u_rows != number_of_joints_)
        {
            std::cout << "The size of the control vector is incorrect" << std::endl;
            return false;
        }
        if (foot_locations_rows != 6)
        {
            std::cout << "The size of the foot location rows is incorrect" << std::endl;
            return false;
        }

        if (hardware_status == 0)
        {


            const VectorViewReadOnly state(x_data, x_rows);
            const Eigen::Vector3d base_position = state.head(3);
            const Eigen::Vector4d base_orientation = state.segment(3, 4);
            const Eigen::VectorXd joint_positions = state.segment(7, number_of_joints_);
            const Eigen::Vector3d base_linear_velocity = state.segment(7 + number_of_joints_, 3);
            const Eigen::Vector3d base_angular_velocity = state.segment(10 + number_of_joints_, 3);
            const Eigen::VectorXd joint_velocities = state.tail(number_of_joints_);

            desired_state_data_.head(7) << base_position, base_orientation;
            desired_state_data_.segment(7, number_of_joints_) = constant_position_controller_.get_desired_joint_positions();
            desired_state_data_.segment(7 + number_of_joints_, 6) << base_linear_velocity, base_angular_velocity;
            desired_state_data_.tail(number_of_joints_) = constant_position_controller_.get_desired_joint_velocities();

            desired_control_data_ = constant_position_controller_.get_desired_joint_torques();
            p_gains_ = constant_position_controller_.get_desired_joint_stiffnesses();
            d_gains_ = constant_position_controller_.get_desired_joint_damping();

            return true;
        }
        else if (hardware_status == 1)
        {
            const VectorViewReadOnly state(x_data, x_rows);
            const Eigen::Vector3d base_position = state.head(3);
            const Eigen::Vector4d base_orientation = state.segment(3, 4);
            const Eigen::VectorXd joint_positions = state.segment(7, number_of_joints_);
            const Eigen::Vector3d base_linear_velocity = state.segment(7 + number_of_joints_, 3);
            const Eigen::Vector3d base_angular_velocity = state.segment(10 + number_of_joints_, 3);
            const Eigen::VectorXd joint_velocities = state.tail(number_of_joints_);

            zmq_controller_.compute(current_time, x_data, x_rows, u_data, u_rows, behavior_status);

            desired_state_data_.head(7) << base_position, base_orientation;
            desired_state_data_.segment(7, number_of_joints_) = zmq_controller_.get_desired_joint_positions();
            desired_state_data_.segment(7 + number_of_joints_, 6) << base_linear_velocity, base_angular_velocity;
            desired_state_data_.tail(number_of_joints_) = zmq_controller_.get_desired_joint_velocities();

            desired_control_data_ = zmq_controller_.get_desired_joint_torques();
            p_gains_ = zmq_controller_.get_desired_joint_stiffnesses();
            d_gains_ = zmq_controller_.get_desired_joint_damping();

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
        std::cout << "Setting home configuration" << std::endl;
        zmq_controller_.setHomeJointConfiguration(configuration_data, rows);
        return constant_position_controller_.setHomeJointConfiguration(configuration_data, rows);
    }

    int ExternalControlImpl::getDebugDataSize()
    {
        return zmq_controller_.get_debug_data_size();
    }

    bool ExternalControlImpl::getDebugData(double* debug_data_to_pack) {
        VectorView debug_data(debug_data_to_pack, zmq_controller_.get_debug_data_size());

        debug_data = zmq_controller_.get_debug_data();

        return true;
    }
}
