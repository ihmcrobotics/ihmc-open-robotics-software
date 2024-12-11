#pragma once

#include "controllers/constant-position-controller.hpp"

namespace ihmc
{
    class ExternalControlImpl
    {
    public:
        explicit ExternalControlImpl(const double default_stiffness, const double default_damping, const int number_of_joints);

        virtual ~ExternalControlImpl() = default;

        ExternalControlImpl(const ExternalControlImpl&) = delete;

        ExternalControlImpl& operator=(const ExternalControlImpl&) = delete;

        ExternalControlImpl(ExternalControlImpl&&) = delete;

        ExternalControlImpl& operator=(ExternalControlImpl&&) = delete;

        bool setHomeJointConfiguration(const double* configuration_data, int rows);

        bool updateRobotState(const double current_time,
                              const double* x_data, int x_rows,
                              const double* u_data, int u_rows,
                              const bool left_in_contact, const bool right_in_contact,
                              const double* foot_locations, int foot_locations_rows,
                              const int hardware_status);

        bool getSolution(double* state_data_to_pack, int state_rows,
                         double* control_data_to_pack, int control_rows,
                         double* p_gains_to_pack, int p_gain_rows,
                         double* d_gains_to_pack, int d_gain_rows) const;

    private:
        ConstantPositionController constant_position_controller_;

        int number_of_joints_;
        Eigen::VectorXd desired_state_data_;
        Eigen::VectorXd desired_control_data_;
        Eigen::VectorXd p_gains_;
        Eigen::VectorXd d_gains_;
    };
}
