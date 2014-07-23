package us.ihmc.atlas.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ArmCalibrationHelper;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.FishEyeDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.DepthDataFilter;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.SCSLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.SCSPointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotJointStatePublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotPosePublisher;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.darpaRoboticsChallenge.sensors.multisense.MultiSenseSensorManager;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.ros.RosMainNode;

public class AtlasSensorSuiteManager implements DRCSensorSuiteManager
{
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final URI rosCoreURI;
   private final DRCRobotSensorInformation sensorInformation;
   private final DRCRobotJointMap jointMap;

   public AtlasSensorSuiteManager(URI rosCoreURI, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation, DRCRobotJointMap jointMap )
   {
      this.rosCoreURI = rosCoreURI;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.jointMap = jointMap;
   }

   @Override
   public void initializeSimulatedSensors(LocalObjectCommunicator scsCommunicator, RobotPoseBuffer robotPoseBuffer,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, DepthDataFilter lidarDataFilter, String sensorURI)
   {
      SCSCameraDataReceiver cameraReceiver = new SCSCameraDataReceiver(robotPoseBuffer, sensorInformation.getCameraParameters(0), scsCommunicator,
            networkingManager, ppsTimestampOffsetProvider);
      
      if(DRCConfigParameters.USE_POINT_CLOUD_INSTEAD_OF_LIDAR)
      {
         new SCSPointCloudDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, sdfFullRobotModel, sensorInformation, scsCommunicator,
               ppsTimestampOffsetProvider, lidarDataFilter);
      } else {
         DRCRobotLidarParameters lidarParams = sensorInformation.getLidarParameters(0);
         new SCSLidarDataReceiver(robotPoseBuffer, scsCommunicator, networkingManager, sdfFullRobotModel, lidarParams, scsCommunicator,
               ppsTimestampOffsetProvider, lidarDataFilter, sensorURI);
      }

      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      {
         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(scsCommunicator, networkingManager, jointMap);
         cameraReceiver.registerCameraListener(armCalibrationHelper);
      }
   }

   @Override
   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, DRCNetworkProcessorNetworkingManager networkingManager,
         SDFFullRobotModel sdfFullRobotModel, ObjectCommunicator objectCommunicator, DepthDataFilter lidarDataFilter, String sensorURI)
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
     
      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(sensorInformation, robotPoseBuffer, rosMainNode, networkingManager,
            sdfFullRobotModel, objectCommunicator, rosNativeNetworkProcessor, ppsTimestampOffsetProvider, lidarDataFilter, sensorURI);

      new FishEyeDataReceiver(robotPoseBuffer, sensorInformation.getCameraParameters(0).getVideoSettings(), rosMainNode, networkingManager,
            DRCSensorParameters.DEFAULT_FIELD_OF_VIEW, ppsTimestampOffsetProvider);
    
      if (DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS)
      {
         new RosRobotPosePublisher(objectCommunicator, rosMainNode, robotPoseBuffer, sensorInformation, "atlas");
         new RosRobotJointStatePublisher(objectCommunicator, rosMainNode, jointMap.getOrderedJointNames(),"atlas");
      }
      
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();     
     
      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      {
         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(objectCommunicator, networkingManager, jointMap);
         multiSenseSensorManager.registerCameraListener(armCalibrationHelper);
      }

   }

}
