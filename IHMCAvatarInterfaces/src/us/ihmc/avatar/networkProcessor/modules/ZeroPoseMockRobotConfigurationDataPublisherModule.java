package us.ihmc.avatar.networkProcessor.modules;

import java.io.IOException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.tools.thread.ThreadTools;

public class ZeroPoseMockRobotConfigurationDataPublisherModule implements Runnable
{
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ZERO_POSE_PRODUCER, new IHMCCommunicationKryoNetClassList());
   private final FullHumanoidRobotModel fullRobotModel;
   private final ForceSensorDefinition[] forceSensorDefinitions;
   private long timeStamp = 0;
   
   public ZeroPoseMockRobotConfigurationDataPublisherModule(final DRCRobotModel robotModel)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      try
      {
         packetCommunicator.connect();
         Thread t = new Thread(this);
         t.start();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }


   public void sendMockRobotConfiguration(long totalNsecs)
   {
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      RobotConfigurationData robotConfigurationData = new RobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel), forceSensorDefinitions, null, imuDefinitions);

      for(int sensorNumber = 0; sensorNumber <  imuDefinitions.length; sensorNumber++)
      {
         IMUPacket imuPacket = robotConfigurationData.getImuPacketForSensor(sensorNumber);
         Vector3D32 linearAcceleration = new Vector3D32();
         Vector3D32 angularVelocity = new Vector3D32();
         Quaternion32 orientation = new Quaternion32();
         imuPacket.set(linearAcceleration, orientation, angularVelocity);
      }
      
      robotConfigurationData.setRobotMotionStatus(RobotMotionStatus.STANDING);
      robotConfigurationData.setTimestamp(totalNsecs);
      Vector3D translation = new Vector3D();
      Quaternion orientation = new Quaternion();
      robotConfigurationData.setRootTranslation(translation);
      robotConfigurationData.setRootOrientation(orientation);
      
      packetCommunicator.send(robotConfigurationData);
   }


   @Override
   public void run()
   {
      while(true)
      {
         sendMockRobotConfiguration(timeStamp);
         timeStamp += 250L * 1000000L;
         ThreadTools.sleep(250);
      }
   }

}
