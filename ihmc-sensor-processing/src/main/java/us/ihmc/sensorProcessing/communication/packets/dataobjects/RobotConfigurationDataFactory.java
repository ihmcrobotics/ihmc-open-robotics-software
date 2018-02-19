package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class RobotConfigurationDataFactory
{
   public static RobotConfigurationData create(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      RobotConfigurationData message = new RobotConfigurationData();
      message.jointNameHash = RobotConfigurationData.calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions);
      return message;
   }
}
