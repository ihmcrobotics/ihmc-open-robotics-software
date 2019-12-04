package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import java.util.List;
import java.util.zip.CRC32;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public class RobotConfigurationDataFactory
{
   public static RobotConfigurationData create(OneDoFJointReadOnly[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      RobotConfigurationData message = new RobotConfigurationData();
      message.setJointNameHash(RobotConfigurationDataFactory.calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions));
      return message;
   }

   public static int calculateJointNameHash(List<? extends OneDoFJointStateReadOnly> jointStates, List<? extends ForceSensorDataReadOnly> forceSensorData,
                                            List<? extends IMUSensorReadOnly> imuSensorData)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointStateReadOnly jointState : jointStates)
      {
         crc.update(jointState.getJointName().getBytes());
      }

      for (ForceSensorDataReadOnly forceSensorOutput : forceSensorData)
      {
         crc.update(forceSensorOutput.getSensorName().getBytes());
      }

      for (IMUSensorReadOnly imuOutput : imuSensorData)
      {
         crc.update(imuOutput.getSensorName().getBytes());
      }

      return (int) crc.getValue();
   }

   public static int calculateJointNameHash(OneDoFJointReadOnly[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointReadOnly joint : joints)
      {
         crc.update(joint.getName().getBytes());
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         crc.update(forceSensorDefinition.getSensorName().getBytes());
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         crc.update(imuDefinition.getName().getBytes());
      }

      return (int) crc.getValue();
   }

   public static void packJointState(RobotConfigurationData robotConfigurationData, OneDoFJointReadOnly[] newJointData)
   {
      robotConfigurationData.getJointAngles().reset();
      robotConfigurationData.getJointVelocities().reset();
      robotConfigurationData.getJointTorques().reset();

      for (int i = 0; i < newJointData.length; i++)
      {
         robotConfigurationData.getJointAngles().add((float) newJointData[i].getQ());
         robotConfigurationData.getJointVelocities().add((float) newJointData[i].getQd());
         robotConfigurationData.getJointTorques().add((float) newJointData[i].getTau());
      }
   }

   public static void packJointState(RobotConfigurationData robotConfigurationData, List<? extends OneDoFJointReadOnly> newJointData)
   {
      robotConfigurationData.getJointAngles().reset();
      robotConfigurationData.getJointVelocities().reset();
      robotConfigurationData.getJointTorques().reset();

      for (int i = 0; i < newJointData.size(); i++)
      {
         robotConfigurationData.getJointAngles().add((float) newJointData.get(i).getQ());
         robotConfigurationData.getJointVelocities().add((float) newJointData.get(i).getQd());
         robotConfigurationData.getJointTorques().add((float) newJointData.get(i).getTau());
      }
   }
}
