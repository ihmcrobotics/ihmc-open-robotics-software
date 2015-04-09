#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/thread/condition_variable.hpp>

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/math/Angle.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>

#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/physics/Collision.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>

#include <stdio.h>
#include <string>

#include "ByteBuffer.h"
#include "TCPServer.h"

const uint32_t BUFFER_SIZE = 1460;  //MTU = 1500, TCP overhead = 40

// pid to freeze robot before controller connects
const double PRE_CONTROLLER_P = 4000.0;
const double PRE_CONTROLLER_I = 5.0;
const double PRE_CONTROLLER_D = 15.0;
const double PRE_CONTROLLER_I_MAX = 50.0;
const double PRE_CONTROLLER_CMD_MAX = 300.0;

namespace gazebo {
class DRCSimIHMCPlugin: public ModelPlugin {
private:

	physics::ModelPtr model;

    event::ConnectionPtr updateConnection;

    std::vector<sensors::ImuSensorPtr> imus;
    std::vector<physics::JointPtr> forceSensors;
    physics::Joint_V joints;
    physics::JointControllerPtr jointController;

    boost::mutex robotControlLock;
    boost::condition_variable condition;

    TCPServer tcpDataServer;
    TCPServer tcpCommandListener;

    bool initialized;
    bool receivedControlMessage;
    int cyclesRemainingTillControlMessage;
    int64_t lastReceivedTimestamp;

    int simulationCyclesPerControlCycle;
    std::vector<double> desiredPositionsOrTorques;
    std::vector<bool> jointsUnderPositionControl;

    ros::NodeHandle* rosNode;

    double gzPhysicsTimeStep;
    double estimatorFrequencyInHz;
    int tcpSendModulus;
    int modulusCounter;

public:
    DRCSimIHMCPlugin() :
        tcpDataServer(1234, BUFFER_SIZE),
        tcpCommandListener(1235, BUFFER_SIZE),
		initialized(false),
        receivedControlMessage(false),
        cyclesRemainingTillControlMessage(0),
        simulationCyclesPerControlCycle(-1),
		tcpSendModulus(1)
    {
        std::cout << "IHMC Plugin Loaded" << std::endl;
    }

    void SendDataToController(int64_t localLastReceivedTimestamp) {
        ByteBuffer data;
        gazebo::common::Time time = this->model->GetWorld()->GetSimTime();
        uint64_t timeStamp = (uint64_t) time.sec * 1000000000ull + (uint64_t) time.nsec;

        data.put(timeStamp);
        data.put(localLastReceivedTimestamp);

        for (unsigned int i = 0; i < joints.size(); i++) {
            physics::JointPtr joint = joints.at(i);
            data.put(joint->GetAngle(0).Radian());
            data.put(joint->GetVelocity(0));
        }

        for (unsigned int i = 0; i < imus.size(); i++) {
            sensors::ImuSensorPtr imu = imus.at(i);

            math::Quaternion imuRotation = imu->GetOrientation();
            math::Vector3 angularVelocity = imu->GetAngularVelocity();
            math::Vector3 linearAcceleration = imu->GetLinearAcceleration();

            data.put(imuRotation.w);
            data.put(imuRotation.x);
            data.put(imuRotation.y);
            data.put(imuRotation.z);

            data.put(angularVelocity.x);
            data.put(angularVelocity.y);
            data.put(angularVelocity.z);

            data.put(linearAcceleration.x);
            data.put(linearAcceleration.y);
            data.put(linearAcceleration.z);
        }

        for (unsigned int i = 0; i < 4; i++) {
            gazebo::physics::JointWrench wrench = forceSensors.at(i)->GetForceTorque((unsigned int) 0);

            data.put(wrench.body2Torque.x);
            data.put(wrench.body2Torque.y);
            data.put(wrench.body2Torque.z);

            data.put(wrench.body2Force.x);
            data.put(wrench.body2Force.y);
            data.put(wrench.body2Force.z);
        }

        tcpDataServer.send(data);
//        if(this->modulusCounter >= this->tcpSendModulus)
//        {
//        	tcpDataServer.send(data);
//        	this->modulusCounter = 0;
//        }

        this->modulusCounter++;
    }

    static bool sortJoints(const physics::JointPtr& a, const physics::JointPtr& b) {
        return a->GetName().compare(b->GetName()) < 0;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf) {
        this->model = _parent;

        this->gzPhysicsTimeStep = model->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
        this->tcpSendModulus = (int)((1.0 / (this->gzPhysicsTimeStep * 1e3)) + 0.5); // TODO replace 1 with Estimator DT in milliseconds
        this->modulusCounter = 3;

        joints = this->model->GetJoints();
        jointController = this->model->GetJointController();
        std::sort(joints.begin(), joints.end(), DRCSimIHMCPlugin::sortJoints);

        this->rosNode = new ros::NodeHandle("");

        sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
        for (unsigned int i = 0; i < sensors.size(); i++) {
            gazebo::sensors::ImuSensorPtr imu = boost::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensors.at(i));
            if (imu) {
                std::cout << imu->GetParentName() << std::endl;
                if (imu->GetParentName() == "atlas::pelvis") {
                    imus.push_back(imu);
                }
            }
        }

        forceSensors.push_back(this->model->GetJoint("l_arm_wrx"));
        forceSensors.push_back(this->model->GetJoint("r_arm_wrx"));
        forceSensors.push_back(this->model->GetJoint("l_leg_akx"));
        forceSensors.push_back(this->model->GetJoint("r_leg_akx"));

        tcpCommandListener.addReadListener(boost::bind(&DRCSimIHMCPlugin::readControlMessage, this, _1, _2));

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DRCSimIHMCPlugin::OnUpdate, this, _1));

        desiredPositionsOrTorques.resize(joints.size(), 0.0);
        jointsUnderPositionControl.resize(joints.size(), 0.0);

        std::map< std::string, physics::JointPtr > jointsByName = jointController->GetJoints();
        int i = 0;
        for (std::map< std::string, physics::JointPtr>::iterator J = jointsByName.begin(); J != jointsByName.end(); J++)
        {
            jointController->SetPositionPID(J->first, common::PID(PRE_CONTROLLER_P, PRE_CONTROLLER_I, PRE_CONTROLLER_D, PRE_CONTROLLER_I_MAX, -PRE_CONTROLLER_I_MAX, PRE_CONTROLLER_CMD_MAX, -PRE_CONTROLLER_CMD_MAX));

            double initialAngle;
            std::string key = "atlas/" + joints.at(i)->GetName() + "/initial_angle";
            this->rosNode->getParam(key, initialAngle);

            joints.at(i)->SetPosition(0, initialAngle);
            joints.at(i)->SetUpperLimit(0, initialAngle + 3.14);
            joints.at(i)->SetLowerLimit(0, initialAngle - 3.14);

            jointController->SetPositionTarget(J->first, initialAngle);

            desiredPositionsOrTorques[i++] = initialAngle;
        }

        this->rosNode->shutdown();
    }

    void OnUpdate(const common::UpdateInfo & info) {
        boost::unique_lock<boost::mutex> lock(robotControlLock);
        {
        	if(!initialized) {
        		if(!receivedControlMessage) {
        			jointController->Update();
        		} else {
        			std::cout << "Unlocking joints. Starting control." << std::endl;
                    initialized = true;
        		}
    			lock.unlock();
    			return;
        	}

            if (cyclesRemainingTillControlMessage == 0) {
                // Block for control message
                while (!receivedControlMessage) {
                    condition.wait(lock);
                }
                cyclesRemainingTillControlMessage = simulationCyclesPerControlCycle;
                receivedControlMessage = false;
            }

            int i = 0;
            std::map< std::string, physics::JointPtr > jointsByName = jointController->GetJoints();
            for (std::map< std::string, physics::JointPtr>::iterator J = jointsByName.begin(); J != jointsByName.end(); J++) {
            	jointController->SetPositionTarget(J->first, desiredPositionsOrTorques[i++]);
            }

            jointController->Update();
            cyclesRemainingTillControlMessage--;
        }

        int64_t localLastReceivedTimestamp = lastReceivedTimestamp;
        lock.unlock();

        DRCSimIHMCPlugin::SendDataToController(localLastReceivedTimestamp);
    }

    void readControlMessage(char* buffer, std::size_t bytes_transffered)
    {
        boost::unique_lock<boost::mutex> lock(robotControlLock);
        {
            int64_t* longBuffer = ((int64_t*) (buffer));

            int estimatorTicksPerControlTick = longBuffer[0];
            lastReceivedTimestamp = longBuffer[1];
            estimatorFrequencyInHz = longBuffer[2];

            int simCyclesFromEstimatorFreq = estimatorTicksPerControlTick / (this->gzPhysicsTimeStep * estimatorFrequencyInHz);
            int simCyclesAssuming1kHz = estimatorTicksPerControlTick / (this->gzPhysicsTimeStep * 1e3);

            simulationCyclesPerControlCycle = simCyclesAssuming1kHz;

            // Pointer arithmetic is evil
            double* jointPositionsOrTorques = (double*) (buffer + 24);

            for (unsigned int i = 0; i < joints.size(); i++) {
                desiredPositionsOrTorques[i] = jointPositionsOrTorques[i];
            }

            receivedControlMessage = true;

            condition.notify_all();
        }

        lock.unlock();
    }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (DRCSimIHMCPlugin)
}
