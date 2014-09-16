package us.ihmc.atlas.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.util.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPointCloudParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ArmCalibrationHelper;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.FishEyeDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataFilter;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosFootstepServiceClient;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.darpaRoboticsChallenge.ros.RosLocalizationUpdateSubscriber;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotJointStatePublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotPosePublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosSCSLidarPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosTfPublisher;
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
   private DepthDataProcessor depthDataProcessor;

   public AtlasSensorSuiteManager(URI rosCoreURI, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation, DRCRobotJointMap jointMap )
   {
      this.rosCoreURI = rosCoreURI;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.jointMap = jointMap;
   }

   @Override
   public void initializeSimulatedSensors(LocalObjectCommunicator scsCommunicator, ObjectCommunicator fieldObjectCommunicator, RobotPoseBuffer robotPoseBuffer,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sdfFullRobotModel, DepthDataFilter lidarDataFilter, URI sensorURI)
   {
      depthDataProcessor = new DepthDataProcessor(networkingManager,lidarDataFilter);
      depthDataProcessor.setTestbed(networkingManager.getControllerCommandHandler().getTestbed());

      DRCRobotCameraParameters leftEyeCameraParams = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      SCSCameraDataReceiver cameraReceiver = new SCSCameraDataReceiver(robotPoseBuffer, leftEyeCameraParams, scsCommunicator, networkingManager,
            ppsTimestampOffsetProvider);

//      if (sensorInformation.getPointCloudParameters().length > 0)
//      {
//         new SCSPointCloudDataReceiver(depthDataProcessor, robotPoseBuffer, scsCommunicator);
//      }
      
      RosMainNode rosMainNode;
      RosTfPublisher tfPublisher;
      
      
      if (DRCConfigParameters.SEND_SIMULATION_DATA_TO_ROS)
      {
         rosMainNode = new RosMainNode(rosCoreURI,
               "darpaRoboticsChallange/networkProcessor", true);

         RosNativeNetworkProcessor rosNativeNetworkProcessor;
         if (RosNativeNetworkProcessor.hasNativeLibrary())
         {
            rosNativeNetworkProcessor = RosNativeNetworkProcessor.getInstance(rosCoreURI.toString());
            rosNativeNetworkProcessor.connect();
         } else
         {
            rosNativeNetworkProcessor = null;
         }

         ROSNativeTransformTools rosTransformProvider = ROSNativeTransformTools.getInstance(sensorURI);
         rosTransformProvider.connect();
         tfPublisher = new RosTfPublisher(rosMainNode);
         new RosRobotPosePublisher(fieldObjectCommunicator, rosMainNode, ppsTimestampOffsetProvider, robotPoseBuffer, sensorInformation, "atlas", tfPublisher);
         new RosRobotJointStatePublisher(fieldObjectCommunicator, rosMainNode, ppsTimestampOffsetProvider, "atlas");
         new RosLocalizationUpdateSubscriber(rosMainNode, fieldObjectCommunicator, ppsTimestampOffsetProvider);
         
         RosFootstepServiceClient rosFootstepServiceClient = new RosFootstepServiceClient(fieldObjectCommunicator, networkingManager, rosMainNode);

         ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
         rosMainNode.execute();
      }
      
      if (sensorInformation.getLidarParameters().length > 0)
      {
         new SCSLidarDataReceiver(depthDataProcessor, robotPoseBuffer, scsCommunicator, ppsTimestampOffsetProvider, sdfFullRobotModel,
               sensorInformation.getLidarParameters());
         
         if (DRCConfigParameters.SEND_SIMULATION_DATA_TO_ROS)
         {
            new RosSCSLidarPublisher(scsCommunicator, rosMainNode, ppsTimestampOffsetProvider, sdfFullRobotModel, sensorInformation.getLidarParameters(), tfPublisher);
         }
      }
      
      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      {
         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(scsCommunicator, networkingManager, jointMap);
         cameraReceiver.registerCameraListener(armCalibrationHelper);
      }
   }

   @Override
   public void initializePhysicalSensors(RobotPoseBuffer robotPoseBuffer, DRCNetworkProcessorNetworkingManager networkingManager,
         SDFFullRobotModel sdfFullRobotModel, ObjectCommunicator objectCommunicator, DepthDataFilter lidarDataFilter, URI sensorURI)
   {
      RosMainNode rosMainNode = new RosMainNode(rosCoreURI, "darpaRoboticsChallange/networkProcessor", true);
      depthDataProcessor = new DepthDataProcessor(networkingManager,lidarDataFilter);
      depthDataProcessor.setTestbed(networkingManager.getControllerCommandHandler().getTestbed());

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
      ROSNativeTransformTools rosTransformProvider = ROSNativeTransformTools.getInstance(sensorURI);
      rosTransformProvider.connect();
     
      DRCRobotCameraParameters leftFishEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.BLACKFLY_LEFT_CAMERA_ID);
      DRCRobotCameraParameters multisenseLeftEyeCameraParameters = sensorInformation.getCameraParameters(AtlasSensorInformation.MULTISENSE_SL_LEFT_CAMERA_ID);
      DRCRobotLidarParameters multisenseLidarParameters = sensorInformation.getLidarParameters(AtlasSensorInformation.MULTISENSE_LIDAR_ID);
      DRCRobotPointCloudParameters multisenseStereoParameters = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);
      
      
      MultiSenseSensorManager multiSenseSensorManager = new MultiSenseSensorManager(depthDataProcessor, rosTransformProvider, robotPoseBuffer,
            rosMainNode, networkingManager, rosNativeNetworkProcessor,
            ppsTimestampOffsetProvider, sensorURI, multisenseLeftEyeCameraParameters,
            multisenseLidarParameters, multisenseStereoParameters);

      FishEyeDataReceiver fishEyeDataReceiver = new FishEyeDataReceiver(robotPoseBuffer, sensorInformation.getCameraParameters(0).getVideoSettings(), rosMainNode, networkingManager,
            DRCSensorParameters.DEFAULT_FIELD_OF_VIEW, ppsTimestampOffsetProvider);
    
      if (DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS)
      {
         RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode);
         RosRobotPosePublisher robotPosePublisher = new RosRobotPosePublisher(objectCommunicator, rosMainNode, ppsTimestampOffsetProvider, robotPoseBuffer, sensorInformation, "atlas", tfPublisher);
         new RosLocalizationUpdateSubscriber(rosMainNode, objectCommunicator, ppsTimestampOffsetProvider);
         multiSenseSensorManager.setRobotPosePublisher(robotPosePublisher);
         new RosRobotJointStatePublisher(objectCommunicator, rosMainNode, ppsTimestampOffsetProvider,"atlas");
      }
      
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosMainNode.execute();     
         
            
      if (DRCConfigParameters.CALIBRATE_ARM_MODE)
      {
         ArmCalibrationHelper armCalibrationHelper = new ArmCalibrationHelper(objectCommunicator, networkingManager, jointMap);
         multiSenseSensorManager.registerCameraListener(armCalibrationHelper);
      }
      
      multiSenseSensorManager.initializeParameterListeners();
      fishEyeDataReceiver.getBlackFlyParameterSetter().initializeParameterListeners();
   }

}
