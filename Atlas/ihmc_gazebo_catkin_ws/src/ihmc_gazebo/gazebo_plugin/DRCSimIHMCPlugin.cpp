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

        std::cout << "local last timestamp = " << localLastReceivedTimestamp << std::endl;

        data.put(timeStamp);
        data.put(localLastReceivedTimestamp);

        for (unsigned int i = 0; i < joints.size(); i++) {
            physics::JointPtr joint = joints.at(i);
            data.put(joint->GetAngle(0).Radian());
            data.put(joint->GetVelocity(0));
            std::cout << "joint " << joint->GetName() << " " << joint->GetAngle(0).Radian() << std::endl;
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
//
//        this->modulusCounter++;
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

        std::string startupParameter = "none";

        this->rosNode->getParam("atlas/startup_mode", startupParameter);

        std::cout << "startupParameter: "<< startupParameter << std::endl;

        sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
        for (unsigned int i = 0; i < sensors.size(); i++) {
            gazebo::sensors::ImuSensorPtr imu = boost::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensors.at(i));
            if (imu) {
                std::cout << imu->GetParentName() << std::endl;
                if (imu->GetParentName() == "atlas::pelvis") {
                    imus.push_back(imu);
                }
            }

            gazebo::sensors::ForceTorqueSensorPtr forcetorque = boost::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(sensors.at(i));
            if (forcetorque) {
                std::cout << "Gazebo bug is fixed. Get force torque from sdf now" << std::endl;
            }
        }

        forceSensors.push_back(this->model->GetJoint("l_arm_wrx"));
        forceSensors.push_back(this->model->GetJoint("r_arm_wrx"));
        forceSensors.push_back(this->model->GetJoint("l_leg_akx"));
        forceSensors.push_back(this->model->GetJoint("r_leg_akx"));

        tcpCommandListener.addReadListener(boost::bind(&DRCSimIHMCPlugin::readControlMessage, this, _1, _2));

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DRCSimIHMCPlugin::OnUpdate, this, _1));

        for (unsigned int i = 0; i < joints.size(); i++) {
            if(joints.at(i)->GetName() != "hokuyo_joint")
            {
            	double initialAngle;
            	std::string key = "robot_initial_configuration/" + joints.at(i)->GetName();
            	bool param = this->rosNode->getParam(key, initialAngle);

                joints.at(i)->SetPosition(0, initialAngle);
                joints.at(i)->SetUpperLimit(0, initialAngle);
                joints.at(i)->SetLowerLimit(0, initialAngle);
            }
        }

        desiredPositionsOrTorques.resize(joints.size(), 0.0);
        jointsUnderPositionControl.resize(joints.size(), 0.0);

        this->rosNode->shutdown();
    }
 public:

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

//        	std::cout << "controlling, cycles remaining = " << cyclesRemainingTillControlMessage << std::endl;

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
            	jointController->SetPositionTarget(J->first, desiredPositionsOrTorques[i]);
            	i++;
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
//        	std::cout << "receiving ctrl message" << std::endl;

        	if (true || bytes_transffered == 32 + joints.size() * 8) {
                int64_t* longBuffer = ((int64_t*) (buffer));

                int estimatorTicksPerControlTick = longBuffer[0];
                lastReceivedTimestamp = longBuffer[1];
                estimatorFrequencyInHz = longBuffer[2];
                int jointPosOrTorqueData = longBuffer[3]; // if the i_th value of the binary rep of this is 1, it's position controlled

                int simCyclesFromEstimatorFreq = estimatorTicksPerControlTick / (this->gzPhysicsTimeStep * estimatorFrequencyInHz);
                int simCyclesAssuming1kHz = estimatorTicksPerControlTick / (this->gzPhysicsTimeStep * 1e3);

                simulationCyclesPerControlCycle = longBuffer[0];
//                std::cout << "simulation cycles per control cycle = " << simulationCyclesPerControlCycle << std::endl;

                for (unsigned int i = 0; i < joints.size(); i++) {
                	jointsUnderPositionControl[i] = jointPosOrTorqueData % 2 == 1;
                	jointPosOrTorqueData /= 2;
                }

                // Pointer arithmetic is evil
                double* jointPositionsOrTorques = (double*) (buffer + 32);
                for (unsigned int i = 0; i < joints.size(); i++) {
                    desiredPositionsOrTorques[i] = jointPositionsOrTorques[i];
                }

                receivedControlMessage = true;

                condition.notify_all();

            } else {
                std::cerr << "Received invalid control packet. Expected length " << (32 + joints.size() * 8) << " bytes, got " << bytes_transffered << std::endl;
            }
        }

        lock.unlock();
    }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (DRCSimIHMCPlugin)
}
