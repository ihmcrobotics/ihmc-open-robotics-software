package us.ihmc.darpaRoboticsChallenge.sensors.multisense;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ArmCalibrationHelper;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraInfoReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.CameraLogger;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraInfoReciever;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.RosCameraReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.LidarFilter;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.lidar.RosLidarDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosNativeNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   private final RosCameraReceiver cameraReceiver;

   public MultiSenseSensorManager(DRCRobotSensorInformation sensorInformation, RobotPoseBuffer sharedRobotPoseBuffer, RosMainNode rosMainNode,
         DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel sharedFullRobotModel, ObjectCommunicator fieldComputerClient,
         RosNativeNetworkProcessor rosNativeNetworkProcessor, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, LidarFilter lidarDataFilter)
   {

      DRCRobotCameraParamaters cameraParamaters = sensorInformation.getPrimaryCameraParamaters();
      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;

      cameraReceiver = new RosCameraReceiver(cameraParamaters, sharedRobotPoseBuffer, cameraParamaters.getVideoSettings(), rosMainNode, networkingManager,
            ppsTimestampOffsetProvider,logger);

      CameraInfoReceiver cameraInfoServer = new RosCameraInfoReciever(sensorInformation.getPrimaryCameraParamaters(), rosMainNode,
            networkingManager.getControllerStateHandler(),logger);
      networkingManager.getControllerCommandHandler().setIntrinsicServer(cameraInfoServer);
      
      double crc = DRCLocalConfigParameters.USING_REAL_HEAD ?  -.0010908f : 0;
      boolean useRosForTransform = true;

      new RosLidarDataReceiver(rosMainNode, sharedRobotPoseBuffer, networkingManager, sharedFullRobotModel, sensorInformation, fieldComputerClient,
            rosNativeNetworkProcessor, ppsTimestampOffsetProvider, lidarDataFilter, useRosForTransform, crc);

      MultiSenseParamaterSetter.setMultisenseResolution(rosMainNode);
      
      if (RosNativeNetworkProcessor.hasNativeLibrary())
      {
         MultiSenseParamaterSetter.setupNativeROSCommunicator(rosNativeNetworkProcessor);
      }
      else
      {
         MultiSenseParamaterSetter.setupMultisenseSpindleSpeedPublisher(rosMainNode);
      }
   }

   public void registerCameraListener(ArmCalibrationHelper armCalibrationHelper)
   {
      cameraReceiver.registerCameraListener(armCalibrationHelper);
      
   }
}
