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
    std::vector<double> desiredTorques;
    std::vector<double> desiredPositions;
    std::vector<bool> jointsUnderPositionControl;

    bool acceptAnyPacketSize; // hack

    ros::NodeHandle* rosNode;

public:
    DRCSimIHMCPlugin() :
        tcpDataServer(1234, BUFFER_SIZE),
        tcpCommandListener(1235, BUFFER_SIZE),
        initialized(false),
        receivedControlMessage(false),
        cyclesRemainingTillControlMessage(0),
        simulationCyclesPerControlCycle(-1),
		acceptAnyPacketSize(true)
    {
        std::cout << "IHMC Plugin Loaded" << std::endl;
    }

    static bool sortJoints(const physics::JointPtr& a, const physics::JointPtr& b) {
        return a->GetName().compare(b->GetName()) < 0;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf) {
        this->model = _parent;

        joints = this->model->GetJoints();
        jointController = this->model->GetJointController();
        std::sort(joints.begin(), joints.end(), DRCSimIHMCPlugin::sortJoints);

        std::cout << "DAVIDE: "<< model->GetName() << std::endl;
        sdf->PrintValues("oops:");
        model->Print("hello: ");

        this->rosNode = new ros::NodeHandle("");

        std::string startupParameter = "none";

        this->rosNode->getParam("atlas/startup_mode", startupParameter);

        this->rosNode->shutdown();

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

        // Nasty initial angles hacks. Works great though. Note alphabetical order of joints!

        double initialAngles[] = { -0.0, 0.0, 0.0, 0.0, //back_bkx back_bky back_bkz hokuyo_joint
                                   0.498, 2.0,   0.1, -1.1,   -0.004, 0.0,  // l_arm_elx l_arm_ely l_arm_shx l_arm_shy l_arm_wrx l_arm_wry
                                   -0.0, -0.28, 0.0, -0.2, 0.0, 0.5,  // l_leg_akx l_leg_aky l_leg_hpx l_leg_hpy l_leg_hpz l_leg_kny
                                   0, //neck_ry
                                   -0.498, 2.0,   -0.1, 1.1,   0.004, 0.0, //r_arm_elx r_arm_ely r_arm_shx r_arm_shy r_arm_wrx r_arm_wry
                                   0.0, -0.28, -0.0, -0.2, 0.0, 0.5 }; // r_leg_akx r_leg_aky r_leg_hpx r_leg_hpy r_leg_hpz r_leg_kny

        /* to stand
        double anklePitchAngle = -0.5; //0.346
        double hipPitchAngle = -0.6;
        double kneeAngle = 1.1;
        double backPitch = 0.1; //0.2658, slightly far forward
         */
        // to sit

        if (startupParameter.find( "car_front") != std::string::npos)
        {
            initialAngles[1] = 0.0; // back pitch
            initialAngles[13] = initialAngles[26] = -1.3; // hip y
            initialAngles[15] = initialAngles[28] = 1.4;  // knee
            initialAngles[11] = initialAngles[24] = -0.2; // ankle y
        }
        else if (startupParameter.find( "car_lateral") != std::string::npos)
        {
            initialAngles[1] = 0.0; // back pitch
            initialAngles[13] = initialAngles[26] = -1.7; // hip y
         //   initialAngles[13] += 0.2;
            initialAngles[15] = initialAngles[28] = 1.5;  // knee
            initialAngles[11] = initialAngles[24] = -0.0; // ankle y
        }
        else if( startupParameter.find( "standing" ) != std::string::npos )
        {
            initialAngles[1] = 0.0; // back pitch
            initialAngles[7]  = initialAngles[20] = -0.0; // shoulder X

            initialAngles[6]  = - 1.0; // shoulder Y
            initialAngles[19] = - initialAngles[6];

            initialAngles[13] = initialAngles[26] = -0.35; // hip y
            initialAngles[15] = initialAngles[28] = 0.7;  // knee
            initialAngles[11] = initialAngles[24] = -0.35; // ankle y
        }




        for (unsigned int i = 0; i < joints.size(); i++) {
            if(joints.at(i)->GetName() != "hokuyo_joint")
            {
                joints.at(i)->SetPosition(0, initialAngles[i]);
                joints.at(i)->SetUpperLimit(0, initialAngles[i] + 3.02);
                joints.at(i)->SetLowerLimit(0, initialAngles[i] - 3.02);
            }
        }

        std::map< std::string, physics::JointPtr > jointsByName =	jointController->GetJoints ();
        int i=0;

        desiredTorques.resize(joints.size(), 0.0);
        desiredPositions.resize(joints.size(), 0.0);
        jointsUnderPositionControl.resize(joints.size(), 0.0);

        for (std::map< std::string, physics::JointPtr>::iterator  J = jointsByName.begin(); J != jointsByName.end(); J++)
        {
            jointController->SetPositionPID( J->first, common::PID( 4000, 5, 15, 50, -50, 300, -300) );
            desiredPositions[i] = J->second->GetAngle(0).Radian ();
            jointController->SetPositionTarget( J->first, desiredPositions[i] );
            i++;
        }
    }
 public:

    void OnUpdate(const common::UpdateInfo & info) {
        boost::unique_lock<boost::mutex> lock(robotControlLock);
        {
            if (!initialized) 
            {
                if (!receivedControlMessage) {

                    jointController->Update();

                } else {
                    // First control tick. Unlock joints
                    std::cout << "Unlocking joints. Starting control." << std::endl;
                    for (unsigned int i = 0; i < joints.size(); i++) {
                        if(joints.at(i)->GetName() != "hokuyo_joint")
                        {
                         //   joints.at(i)->SetEffortLimit(0, -1);
                           // joints.at(i)->SetDamping(0, 0.1);
                        }
                    }

					//jointController->Reset();
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

          /*  for (unsigned int i = 0; i < joints.size(); i++) {
            	joints.at(i)->SetPosition(0, desiredPositions[i]);
            }*/

            int i=0;
            std::map< std::string, physics::JointPtr > jointsByName =	jointController->GetJoints ();

           // std::cout << "-----------"<<  std::endl;
            for (std::map< std::string, physics::JointPtr>::iterator  J = jointsByName.begin(); J != jointsByName.end(); J++)
            {
                jointController->SetPositionTarget( J->first, desiredPositions[i] );
                printf("%.3f ",  desiredPositions[i]  );
                i++;
            }
            printf("\n");

            jointController->Update();

            /*for (unsigned int i = 0; i < joints.size(); i++) {
            	if(jointsUnderPositionControl[i]) {
            		joints.at(i)->SetPosition(0, desiredPositions[i]);
            		jointController->Update();
            	} else {
            		jointController->Reset();
                    joints.at(i)->SetForce(0, desiredTorques[i]);
            	}
            }*/
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

    void readControlMessage(char* buffer, std::size_t bytes_transffered) 
    {
        boost::unique_lock<boost::mutex> lock(robotControlLock);
        {
            if (acceptAnyPacketSize || bytes_transffered == 24 + joints.size() * 8) {

            	int64_t* longBuffer = ((int64_t*) (buffer));
                simulationCyclesPerControlCycle = longBuffer[0];
                lastReceivedTimestamp = longBuffer[1];
                int64_t positionControlData = longBuffer[2]; // if the i_th entry of this value in binary is 1, joint i position controlled

                for (unsigned int i = 0; i < joints.size(); i++) {
                	jointsUnderPositionControl[i] = positionControlData % 2 == 1;
                	positionControlData /= 2;
                }

                // Pointer arithmetic is evil
                // ^ this is a fact
                double* jointData = (double*) (buffer + 24);
                for (unsigned int i = 0; i < joints.size(); i++) {
                    if(jointsUnderPositionControl[i]) {
                    	desiredPositions[i] = jointData[i];
                   // 	std::cout << "SENT: "<<desiredPositions[i] << std::endl;
                    } else {
                    	desiredTorques[i] = jointData[i];
                    }
                }

                std::cout << "" << std::endl;

                receivedControlMessage = true;
                condition.notify_all();

            } else {
                std::cerr << "Received invalid control packet. Expected length " << (24 + joints.size() * 8) << ", got " << bytes_transffered << std::endl;
            }
        }
        lock.unlock();
    }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (DRCSimIHMCPlugin)
}
