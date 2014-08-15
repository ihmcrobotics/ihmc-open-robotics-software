package us.ihmc.darpaRoboticsChallenge.sensors;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataFilter;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;

public interface DRCSensorSuiteManager
{
   public void initializeSimulatedSensors(LocalObjectCommunicator scsCommunicator, ObjectCommunicator fieldComputerClient, RobotPoseBuffer robotPoseBuffer,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, DepthDataFilter lidarDataFilter, String sensorURI);

   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, DRCNetworkProcessorNetworkingManager networkingManager,
         SDFFullRobotModel sdfFullRobotModel, ObjectCommunicator objectCommunicator, DepthDataFilter lidarDataFilter, String sensorURI);

}
