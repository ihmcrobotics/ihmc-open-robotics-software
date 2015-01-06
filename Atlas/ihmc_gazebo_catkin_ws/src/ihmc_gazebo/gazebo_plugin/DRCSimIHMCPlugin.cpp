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

#include <sdf/sdf.hh>

#include <stdio.h>

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

	boost::mutex robotControlLock;
	boost::condition_variable condition;

	TCPServer tcpDataServer;
	TCPServer tcpCommandListener;

	bool initialized;
	bool receivedControlMessage;
	int cyclesRemainingTillControlMessage;
	int64_t lastReceivedTimestamp;

	int simulationCyclesPerControlCycle;
	std::vector<double> desiredTorques;


public:
	DRCSimIHMCPlugin() :
			tcpDataServer(1234, BUFFER_SIZE), tcpCommandListener(1235, BUFFER_SIZE), initialized(false), receivedControlMessage(false), cyclesRemainingTillControlMessage(0), simulationCyclesPerControlCycle(-1)
	{
		std::cout << "IHMC Plugin Loaded" << std::endl;
	}

	static bool sortJoints(const physics::JointPtr& a, const physics::JointPtr& b) {
		return a->GetName().compare(b->GetName()) < 0;
	}

	void Load(physics::ModelPtr _parent, sdf::ElementPtr /* _sdf */) {
		this->model = _parent;

		joints = this->model->GetJoints();
		std::sort(joints.begin(), joints.end(), DRCSimIHMCPlugin::sortJoints);



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

		// Nasty initial angles hacks. Works great though. Note alphabetical order of joints!
		//back_bkx back_bky back_bkz hokuyo_joint l_arm_elx l_arm_ely l_arm_shx l_arm_shy l_arm_wrx l_arm_wry l_leg_akx l_leg_aky l_leg_hpx l_leg_hpy l_leg_hpz l_leg_kny neck_ry r_arm_elx r_arm_ely r_arm_shx r_arm_shy r_arm_wrx r_arm_wry r_leg_akx r_leg_aky r_leg_hpx r_leg_hpy r_leg_hpz r_leg_kny
		double initialAngles[] = { 0.0, 0.0, 0.0, 0.0, 0.498, 2.0, -1.3, 0.3, -0.004, 0.0, -0.062, -0.276, 0.062, -0.233, 0.0, 0.518, 0, -0.498, 2.0, 1.3, 0.3, 0.004, 0.0, 0.062, -0.276, -0.062, -0.233, 0.0, 0.518 };

		double anklePitchAngle = -0.346;
		double hipPitchAngle = -0.663;
		double kneeAngle = 1.008;
		double backPitch = 0.2798;

		double ankleRoll = 0.062;
		initialAngles[10] = -ankleRoll; //left ankle x
		initialAngles[23] = ankleRoll; //right ankle x

		initialAngles[1] = backPitch; // back pitch
		initialAngles[11] = initialAngles[24] = anklePitchAngle; // ankle y
		initialAngles[13] = initialAngles[26] = hipPitchAngle; // hip y
		initialAngles[15] = initialAngles[28] = kneeAngle; // knee

		for (unsigned int i = 0; i < joints.size(); i++) {
			if(joints.at(i)->GetName() != "hokuyo_joint")
			{
				joints.at(i)->SetPosition(0, initialAngles[i]);
				joints.at(i)->SetUpperLimit(0, initialAngles[i]);
				joints.at(i)->SetLowerLimit(0, initialAngles[i]);
			}
		}

		desiredTorques.resize(joints.size(), 0.0);
	}
public:

	void OnUpdate(const common::UpdateInfo & info) {
		boost::unique_lock<boost::mutex> lock(robotControlLock);
		{
			if (!initialized) {
				if (!receivedControlMessage) {
					// Waiting for controller to connect, do not actively control robot
				} else {
					// First control tick. Unlock joints
					std::cout << "Unlocking joints. Starting control." << std::endl;
					for (unsigned int i = 0; i < joints.size(); i++) {
						if(joints.at(i)->GetName() != "hokuyo_joint")
						{
							joints.at(i)->SetLowerLimit(0, -3.14);
							joints.at(i)->SetUpperLimit(0, 3.14);
							joints.at(i)->SetEffortLimit(0, -1);
							joints.at(i)->SetDamping(0, 0.1);
						}
					}

					initialized = true;
				}
				lock.unlock();	// make sure to unlock
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
			for (unsigned int i = 0; i < joints.size(); i++) {
				joints.at(i)->SetForce(0, desiredTorques[i]);
			}
			cyclesRemainingTillControlMessage--;
		}

		int64_t localLastReceivedTimestamp = lastReceivedTimestamp;
		lock.unlock();

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

			//			data.put((double) 1);
			//			data.put((double) 0);
			//			data.put((double) 0);
			//			data.put((double) 0);
			//			data.put((double) 0);
			//			data.put((double) 0);
			//			data.put((double) 0);
			//			data.put((double) 0);
			//			data.put((double) 0);
			//			data.put((double) 0);

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
	}

	void readControlMessage(char* buffer, std::size_t bytes_transffered) {
		boost::unique_lock<boost::mutex> lock(robotControlLock);
		{
			if (bytes_transffered == 16 + joints.size() * 8) {
				int64_t* longBuffer = ((int64_t*) (buffer));
				simulationCyclesPerControlCycle = longBuffer[0];
				lastReceivedTimestamp = longBuffer[1];

				// Pointer arithmetic is evil
				double* jointTorques = (double*) (buffer + 16);
				for (unsigned int i = 0; i < joints.size(); i++) {
					desiredTorques[i] = jointTorques[i];
				}

				receivedControlMessage = true;
				condition.notify_all();

			} else {
				std::cerr << "Received invalid control packet. Expected length " << (16 + joints.size() * 8) << ", got " << bytes_transffered << std::endl;
			}

		}
		lock.unlock();

	}

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (DRCSimIHMCPlugin)
}
