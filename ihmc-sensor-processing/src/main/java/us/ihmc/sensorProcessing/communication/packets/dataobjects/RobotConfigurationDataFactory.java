package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import java.util.List;
import java.util.zip.CRC32;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class RobotConfigurationDataFactory
{
   public static RobotConfigurationData create(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      RobotConfigurationData message = new RobotConfigurationData();
      message.setJointNameHash(RobotConfigurationDataFactory.calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions));
      return message;
   }

   public static int calculateJointNameHash(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointBasics joint : joints)
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

   public static void packJointState(RobotConfigurationData robotConfigurationData, OneDoFJointBasics[] newJointData)
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

   public static void packJointState(RobotConfigurationData robotConfigurationData, List<? extends OneDoFJointBasics> newJointData)
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
