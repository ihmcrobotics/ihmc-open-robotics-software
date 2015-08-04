package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules;

import java.io.IOException;

import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.IMUPacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.robotics.humanoidRobot.model.RobotMotionStatus;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModelUtils;

public class ZeroPoseMockRobotConfigurationDataPublisherModule implements Runnable
{
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ZERO_POSE_PRODUCER, new IHMCCommunicationKryoNetClassList());
   private final SDFFullRobotModel fullRobotModel;
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
         Vector3f linearAcceleration = new Vector3f();
         Vector3f angularVelocity = new Vector3f();
         Quat4f orientation = new Quat4f();
         imuPacket.set(linearAcceleration, orientation, angularVelocity);
      }
      
      robotConfigurationData.setRobotMotionStatus(RobotMotionStatus.STANDING);
      robotConfigurationData.setTimestamp(totalNsecs);
      Vector3d translation = new Vector3d();
      Quat4d orientation = new Quat4d();
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
