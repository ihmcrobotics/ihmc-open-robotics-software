package us.ihmc.sensorProcessing.communication.producers;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

class RobotConfigurationDataTest
{
   @Test
   void testSetRobotConfigurationDataSequenceId()
   {
      OneDoFJointBasics[] joints = new OneDoFJointBasics[31];
      RigidBodyBasics body = new RigidBody("aap", ReferenceFrame.getWorldFrame());
      for (int i = 0; i < joints.length; i++)
      {
         joints[i] = new RevoluteJoint("noot", body, new RigidBodyTransform(), new Vector3D(1, 0, 0));
      }

      RigidBodyBasics body2 = new RigidBody("mies", joints[0], new Matrix3D(), 0.0, new RigidBodyTransform());
      IMUDefinition imuSensorDefinitions[] = new IMUDefinition[3];
      for (int i = 0; i < imuSensorDefinitions.length; i++)
      {
         imuSensorDefinitions[i] = new IMUDefinition("fake_imu" + i, body2, new RigidBodyTransform());
      }

      ForceSensorDefinition forceSensorDefinitions[] = new ForceSensorDefinition[4];
      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         ReferenceFrame sensorFrame = ForceSensorDefinition.createSensorFrame("wim", body2, new RigidBodyTransform());
         forceSensorDefinitions[i] = new ForceSensorDefinition("wim", body2, sensorFrame);
      }

      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuSensorDefinitions);

      long sequenceId = 11;

      MessageTools.setRobotConfigurationDataSequenceId(robotConfigurationData, sequenceId);

      assertEquals(robotConfigurationData.getSequenceId(), sequenceId);

      for (int i = 0; i < robotConfigurationData.getImuSensorData().size(); i++)
      {
         assertEquals(robotConfigurationData.getImuSensorData().get(i).getSequenceId(), sequenceId);
      }
      for (int i = 0; i < robotConfigurationData.getForceSensorData().size(); i++)
      {
         assertEquals(robotConfigurationData.getForceSensorData().get(i).getSequenceId(), sequenceId);
      }
   }

}
