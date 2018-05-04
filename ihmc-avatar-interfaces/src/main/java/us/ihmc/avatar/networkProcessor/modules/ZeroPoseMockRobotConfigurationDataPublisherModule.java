package us.ihmc.avatar.networkProcessor.modules;

import java.io.IOException;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

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
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel), forceSensorDefinitions, imuDefinitions);

      for(int sensorNumber = 0; sensorNumber <  imuDefinitions.length; sensorNumber++)
      {
         robotConfigurationData.getImuSensorData().add();
      }
      
      robotConfigurationData.setRobotMotionStatus(RobotMotionStatus.STANDING.toByte());
      robotConfigurationData.setTimestamp(totalNsecs);
      Vector3D translation = new Vector3D();
      Quaternion orientation = new Quaternion();
      robotConfigurationData.getRootTranslation().set(translation);
      robotConfigurationData.getRootOrientation().set(orientation);
      
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
