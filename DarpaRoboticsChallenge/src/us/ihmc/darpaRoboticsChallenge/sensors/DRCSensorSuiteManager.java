package us.ihmc.darpaRoboticsChallenge.sensors;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.RobotBoundingBoxes;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;

public interface DRCSensorSuiteManager
{
   public void initializeSimulatedSensors(LocalObjectCommunicator scsCommunicator, RobotPoseBuffer robotPoseBuffer,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, RobotBoundingBoxes robotBoundingBoxes);

   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, DRCNetworkProcessorNetworkingManager networkingManager,
         SDFFullRobotModel sdfFullRobotModel, RobotBoundingBoxes robotBoundingBoxes, ObjectCommunicator objectCommunicator);

}
