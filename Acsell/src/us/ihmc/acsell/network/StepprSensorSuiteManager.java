package us.ihmc.acsell.network;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;

public class StepprSensorSuiteManager implements DRCSensorSuiteManager
{

   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsCommunicator, PacketCommunicator fieldComputerClient, RobotPoseBuffer robotPoseBuffer,
         AbstractNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, DepthDataFilter lidarDataFilter, URI sensorURI)
   {

   }

   @Override
   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, AbstractNetworkProcessorNetworkingManager networkingManager,
         SDFFullRobotModel sdfFullRobotModel, PacketCommunicator objectCommunicator, DepthDataFilter lidarDataFilter, URI sensorURI)
   {
      // Faking the LIDAR
      new PointCloudDataReceiver(robotPoseBuffer, networkingManager, sdfFullRobotModel, lidarDataFilter);
   }

}
