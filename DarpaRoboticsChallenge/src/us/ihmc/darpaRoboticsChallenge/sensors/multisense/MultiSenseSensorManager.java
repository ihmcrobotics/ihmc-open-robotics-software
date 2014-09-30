package us.ihmc.darpaRoboticsChallenge.sensors.multisense;

import java.net.URI;

import org.ros.node.parameter.ParameterTree;

import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPointCloudParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ArmCalibrationHelper;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraLogger;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotPosePublisher;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   private RosCameraReceiver cameraReceiver;

   private final RobotPoseBuffer sharedRobotPoseBuffer;
   private final RosMainNode rosMainNode;
   private final DRCNetworkProcessorNetworkingManager networkingManager;
   private final RosNativeNetworkProcessor rosNativeNetworkProcessor;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private static final double lidarCRC = -.0010908f;

   private final URI sensorURI;

   private final DRCRobotCameraParameters cameraParamaters;
   private final DRCRobotLidarParameters lidarParamaters;

   private final DepthDataProcessor depthDataProcessor;

   private final ROSNativeTransformTools rosTransformProvider;

   private final DRCRobotPointCloudParameters stereoParamaters;

   private ParameterTree params;

   private MultisenseLidarDataReceiver multisenseLidarDataReceiver;

   private MultiSenseParamaterSetter multiSenseParamaterSetter;

   public MultiSenseSensorManager(DepthDataProcessor depthDataProcessor, ROSNativeTransformTools rosTransformProvider, RobotPoseBuffer sharedRobotPoseBuffer,
         RosMainNode rosMainNode, DRCNetworkProcessorNetworkingManager networkingManager, RosNativeNetworkProcessor rosNativeNetworkProcessor,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider, URI sensorURI, DRCRobotCameraParameters cameraParamaters,
         DRCRobotLidarParameters lidarParamaters, DRCRobotPointCloudParameters stereoParamaters)
   {
      this.depthDataProcessor = depthDataProcessor;
      this.rosTransformProvider = rosTransformProvider;
      this.lidarParamaters = lidarParamaters;
      this.stereoParamaters = stereoParamaters;
      this.sharedRobotPoseBuffer = sharedRobotPoseBuffer;
      this.cameraParamaters = cameraParamaters;
      this.rosMainNode = rosMainNode;
      this.networkingManager = networkingManager;
      this.rosNativeNetworkProcessor = rosNativeNetworkProcessor;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorURI = sensorURI;
      registerCameraReceivers();
      registerLidarReceivers();
      multiSenseParamaterSetter = new MultiSenseParamaterSetter(rosMainNode, networkingManager);
      setMultiseSenseParams(lidarParamaters.getLidarSpindleVelocity());
      networkingManager.getControllerCommandHandler().setMultiSenseSensorManager(multiSenseParamaterSetter);

   }

   public void initializeParameterListeners()
   {

      System.out.println("initialise parameteres--------------------------------------------------------------------------------");
      multiSenseParamaterSetter.initializeParameterListeners(); 
   }

   private void setMultiseSenseParams(double lidarSpindleVelocity)
   {
      multiSenseParamaterSetter.setMultisenseResolution(rosMainNode);

      if (RosNativeNetworkProcessor.hasNativeLibrary())
      {
         multiSenseParamaterSetter.setupNativeROSCommunicator(rosNativeNetworkProcessor, lidarSpindleVelocity);
      }
      else
      {
         multiSenseParamaterSetter.setupMultisenseSpindleSpeedPublisher(rosMainNode, lidarSpindleVelocity);
      }
   }

   private void registerLidarReceivers()
   {
      this.multisenseLidarDataReceiver = new MultisenseLidarDataReceiver(depthDataProcessor, rosTransformProvider, ppsTimestampOffsetProvider,
            sharedRobotPoseBuffer, rosMainNode, lidarParamaters);
   }

   private void registerCameraReceivers()
   {
      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
      cameraReceiver = new RosCameraReceiver(cameraParamaters, sharedRobotPoseBuffer, rosMainNode, networkingManager,
            ppsTimestampOffsetProvider, logger, sensorURI);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(cameraParamaters, rosMainNode, networkingManager.getControllerStateHandler(), logger);

      networkingManager.getControllerCommandHandler().setIntrinsicServer(cameraInfoServer);
   }

   public void registerCameraListener(ArmCalibrationHelper armCalibrationHelper)
   {
      cameraReceiver.registerCameraListener(armCalibrationHelper);

   }

   public void setRobotPosePublisher(RosRobotPosePublisher robotPosePublisher)
   {
      multisenseLidarDataReceiver.setRobotPosePublisher(robotPosePublisher);
      cameraReceiver.setRobotPosePublisher(robotPosePublisher);
   }
}
