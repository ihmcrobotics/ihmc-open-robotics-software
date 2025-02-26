#include "zmq-controller.hpp"

#include <iostream>

namespace ihmc
{
    ZMQController::ZMQController(const double default_stiffness, const double default_damping, const int number_of_joints)
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

        debug_data_.resize(debug_data_size_);
        debug_data_.setZero();

        // Init ZMQ
        context = zmq::context_t(1);
        socket_cpp = zmq::socket_t(context, zmq::socket_type::req);
        int timeout = 500;
        socket_cpp.setsockopt(ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
        socket_cpp.setsockopt(ZMQ_SNDTIMEO, &timeout, sizeof(timeout));

        socket_cpp.bind("tcp://*:5555");
    }

    void ZMQController::stopSocket() {
        socket_cpp.close();
    }

    void ZMQController::startSocket() {
        socket_cpp = zmq::socket_t(context, zmq::socket_type::req);
        socket_cpp.bind("tcp://*:5555");
    }

    void ZMQController::resize(const double default_stiffness, const double default_damping, const int number_of_joints)
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

    bool ZMQController::setHomeJointConfiguration(const double* configuration_data, int configuration_rows)
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

    void ZMQController::compute(const double current_time,
                               const double* x_data, int x_rows,
                               const double* u_data, int u_rows,
                               int behavior_status) {
        // Send zmq message
        std::vector<double> zmq_outgoing_msg(1 + x_rows + u_rows + 1);
        zmq_outgoing_msg[0] = current_time;
        std::copy(x_data, x_data + x_rows, zmq_outgoing_msg.begin() + 1);
        std::copy(u_data, u_data + u_rows, zmq_outgoing_msg.begin() + 1 + x_rows);
        zmq_outgoing_msg[1 + x_rows + u_rows] = behavior_status;
        socket_cpp.send(zmq_outgoing_msg.data(), zmq_outgoing_msg.size() * sizeof(double));

        // Receive reply
        zmq::message_t zmq_incoming_msg;
        socket_cpp.recv(zmq_incoming_msg);
        const double* receivedData = static_cast<double*>(zmq_incoming_msg.data());
        size_t receivedSize = zmq_incoming_msg.size() / sizeof(double);
        std::vector<double> values(receivedData, receivedData + receivedSize);

        // Pull out reply into elements it is ([t; x_d; u_d; Kp; Kd])
        Eigen::VectorXd data = Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
        home_configuration_ = data.segment<23>(1 + 7); // pull joint position out of x_d
        desired_joint_velocities_ = data.segment<23>(1 + 30 + 6); // pull joint velocities out of x_d
        desired_joint_torques_= data.segment<23>(1 + 30 + 29);
        desired_joint_stiffnesses_ = data.segment<23>(1 + 30 + 29 + 23);
        desired_joint_damping_ = data.segment<23>(1 + 30 + 29 + 23 + 23);
        debug_data_ = data.segment(1 + 30 + 29 + 23 + 23 + 23, debug_data_size_);

    }

    Eigen::VectorXd ZMQController::get_desired_joint_positions() const
    {
        return home_configuration_;
    }

    Eigen::VectorXd ZMQController::get_desired_joint_velocities() const
    {
        return desired_joint_velocities_;
    }

    Eigen::VectorXd ZMQController::get_desired_joint_torques() const
    {
        return desired_joint_torques_;
    }

    Eigen::VectorXd ZMQController::get_desired_joint_stiffnesses() const
    {
        return desired_joint_stiffnesses_;
    }

    Eigen::VectorXd ZMQController::get_desired_joint_damping() const
    {
        return desired_joint_damping_;
    }

    int ZMQController::get_debug_data_size()
    {
        return debug_data_size_;
    }

    Eigen::VectorXd ZMQController::get_debug_data() const
    {
        return debug_data_;
    }

}