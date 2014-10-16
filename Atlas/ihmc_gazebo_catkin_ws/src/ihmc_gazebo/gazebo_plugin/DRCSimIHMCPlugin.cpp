#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

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

	TCPServer tcpDataServer;
	TCPServer tcpCommandListener;

public:
	DRCSimIHMCPlugin() :
			tcpDataServer(1234, BUFFER_SIZE), tcpCommandListener(1235, BUFFER_SIZE) {
		std::cout << "IHMC Plugin Loaded" << std::endl;
	}

	static bool sortJoints(const physics::JointPtr& a, const physics::JointPtr& b) {
		return a->GetName().compare(b->GetName()) < 0;
	}

	void Load(physics::ModelPtr _parent, sdf::ElementPtr /* _sdf */) {
		this->model = _parent;

		joints = this->model->GetJoints();
		std::sort(joints.begin(), joints.end(), DRCSimIHMCPlugin::sortJoints);

		for (unsigned int i = 0; i < joints.size(); i++) {
			const boost::shared_ptr<physics::Joint> joint = joints.at(i);
			std::cout << joint->GetName() << " ";
		}
		std::cout << std::endl;

		sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
		for (unsigned int i = 0; i < sensors.size(); i++) {
			std::cout << sensors.at(i)->GetName() << std::endl;

			gazebo::sensors::ImuSensorPtr imu = boost::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensors.at(i));
			if (imu) {
				std::cout << imu->GetParentName() << std::endl;
				if (imu->GetParentName() == "atlas::pelvis") {
					std::cout << "Adding imu " << imu->GetName() << std::endl;
					imus.push_back(imu);
				}
			}

			gazebo::sensors::ForceTorqueSensorPtr forcetorque = boost::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(sensors.at(i));
			if (forcetorque) {
				std::cout << "Ignoring force torque sensor" << std::endl;
			}
		}

		tcpCommandListener.addReadListener(boost::bind(&DRCSimIHMCPlugin::readControlMessage, this, _1, _2));

		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DRCSimIHMCPlugin::OnUpdate, this, _1));
	}

public:

	void OnUpdate(const common::UpdateInfo & info) {
		ByteBuffer data;
		gazebo::common::Time time = this->model->GetWorld()->GetSimTime();
		uint64_t timeStamp = 12345;//(uint64_t) time.sec * 1000000000ull + (uint64_t) time.nsec;

		data.put(timeStamp);

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

		for(unsigned int i = 0; i < 4; i++)
		{
			data.put((double) 0);
			data.put((double) 0);
			data.put((double) 0);
			data.put((double) 0);
			data.put((double) 0);
			data.put((double) 0);
		}

		tcpDataServer.send(data);
	}

	void readControlMessage(char* buffer, std::size_t bytes_transffered) {
		boost::unique_lock<boost::mutex> lock(robotControlLock);
		{

		}
		lock.unlock();
	}

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (DRCSimIHMCPlugin)
}
