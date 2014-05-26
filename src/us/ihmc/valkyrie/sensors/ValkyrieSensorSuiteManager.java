package us.ihmc.valkyrie.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.GazeboLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.IbeoPointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.RobotBoundingBoxes;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.SCSLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.SCSPointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.ros.RosMainNode;

public class ValkyrieSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final URI rosCoreURI;
   private final DRCRobotSensorInformation sensorInformation;

   public ValkyrieSensorSuiteManager(URI rosCoreURI, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation)
   {
      this.rosCoreURI = rosCoreURI;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
   }

   public void initializeSimulatedSensors(LocalObjectCommunicator scsCommunicator, RobotPoseBuffer robotPoseBuffer,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, RobotBoundingBoxes robotBoundingBoxes)
   {
      new SCSCameraDataReceiver(robotPoseBuffer, sensorInformation.getPrimaryCameraParamaters(), scsCommunicator, networkingManager, ppsTimestampOffsetProvider);
      if(DRCConfigParameters.USE_POINT_CLOUD_INSTEAD_OF_LIDAR)
      {
         new SCSPointCloudDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, sdfFullRobotModel, robotBoundingBoxes, sensorInformation, scsCommunicator,
               ppsTimestampOffsetProvider);
      } else {
         new SCSLidarDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, sdfFullRobotModel, robotBoundingBoxes, sensorInformation, scsCommunicator,
               ppsTimestampOffsetProvider);
      }
   }

   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, DRCNetworkProcessorNetworkingManager networkingManager,
         SDFFullRobotModel sdfFullRobotModel, RobotBoundingBoxes robotBoundingBoxes, ObjectCommunicator objectCommunicator)
   {
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, "darpaRoboticsChallange/networkProcessor");

      RosNativeNetworkProcessor rosNativeNetworkProcessor;
      if (RosNativeNetworkProcessor.hasNativeLibrary())
      {
         rosNativeNetworkProcessor = RosNativeNetworkProcessor.getInstance(rosCoreURI.toString());
         rosNativeNetworkProcessor.connect();
      }
      else
      {
         rosNativeNetworkProcessor = null;
      }

      DRCRobotCameraParamaters cameraParamaters = sensorInformation.getPrimaryCameraParamaters();

      new RosCameraReceiver(cameraParamaters, robotPoseBuffer, cameraParamaters.getVideoSettings(), rosMainNode, networkingManager, ppsTimestampOffsetProvider);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(cameraParamaters, rosMainNode, networkingManager.getControllerStateHandler());
      networkingManager.getControllerCommandHandler().setIntrinsicServer(cameraInfoServer);

      new IbeoPointCloudDataReceiver(rosMainNode, robotPoseBuffer, networkingManager, sdfFullRobotModel, robotBoundingBoxes, sensorInformation, objectCommunicator, rosNativeNetworkProcessor, ppsTimestampOffsetProvider);

      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);

      rosMainNode.execute();
   }
}
