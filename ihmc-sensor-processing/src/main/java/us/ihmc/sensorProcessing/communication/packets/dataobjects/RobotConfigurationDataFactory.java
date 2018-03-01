package us.ihmc.sensorProcessing.communication.packets.dataobjects;

import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class RobotConfigurationDataFactory
{
   public static RobotConfigurationData create(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions, AuxiliaryRobotData auxiliaryRobotData,
                                               IMUDefinition[] imuDefinitions)
   {
      RobotConfigurationData message = new RobotConfigurationData();
      message.jointAngles = new float[joints.length];
      message.jointVelocities = new float[joints.length];
      message.jointTorques = new float[joints.length];
      message.momentAndForceDataAllForceSensors = new float[forceSensorDefinitions.length][Wrench.SIZE];

      message.imuSensorData = new IMUPacket[imuDefinitions.length];
      for (int sensorNumber = 0; sensorNumber < message.imuSensorData.length; sensorNumber++)
      {
         message.imuSensorData[sensorNumber] = new IMUPacket();
      }

      message.jointNameHash = RobotConfigurationData.calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions);
      message.auxiliaryRobotData = auxiliaryRobotData;
      return message;
   }
}
