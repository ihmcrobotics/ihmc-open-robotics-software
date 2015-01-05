package us.ihmc.darpaRoboticsChallenge.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;

public interface DRCSensorSuiteManager
{
   public void initializeSimulatedSensors(PacketCommunicator scsCommunicator, PacketCommunicator fieldComputerClient, RobotPoseBuffer robotPoseBuffer,
                                          AbstractNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, DepthDataFilter lidarDataFilter, URI sensorURI);

   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, AbstractNetworkProcessorNetworkingManager networkingManager,
                                         SDFFullRobotModel sdfFullRobotModel, PacketCommunicator objectCommunicator, DepthDataFilter lidarDataFilter, URI sensorURI);

}
