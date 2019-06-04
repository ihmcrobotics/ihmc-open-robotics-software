package us.ihmc.avatar.sensors.multisense;

import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.CameraLogger;
import us.ihmc.ihmcPerception.camera.RosCameraCompressedImageReceiver;
import us.ihmc.ihmcPerception.camera.VideoPacketHandler;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   public static boolean LOG_PRIMARY_CAMERA_IMAGES = false;

   private CameraDataReceiver cameraReceiver;

   private final FullRobotModelFactory fullRobotModelFactory;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private final AvatarRobotCameraParameters cameraParameters;

   private MultiSenseParamaterSetter multiSenseParameterSetter;

   public MultiSenseSensorManager(FullRobotModelFactory sdfFullRobotModelFactory, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
                                  RosMainNode rosMainNode, Ros2Node ros2Node, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
                                  AvatarRobotCameraParameters cameraParameters, AvatarRobotLidarParameters lidarParameters,
                                  AvatarRobotPointCloudParameters stereoParameters, boolean setROSParameters)
   {
      this.fullRobotModelFactory = sdfFullRobotModelFactory;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.cameraParameters = cameraParameters;
      this.rosMainNode = rosMainNode;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;

      boolean rosOnline = false;

      while (!rosOnline)
      {
         registerCameraReceivers(ros2Node);

         if (setROSParameters)
         {
            multiSenseParameterSetter = new MultiSenseParamaterSetter(rosMainNode, ros2Node);
            rosOnline = setMultiseSenseParams(lidarParameters.getLidarSpindleVelocity());
         }
         else
         {
            multiSenseParameterSetter = null;
            rosOnline = true;
         }
      }
   }

   public void initializeParameterListeners()
   {
      if (multiSenseParameterSetter != null)
      {
         multiSenseParameterSetter.initializeParameterListeners();
      }
   }

   private boolean setMultiseSenseParams(double lidarSpindleVelocity)
   {
      if (multiSenseParameterSetter != null)
      {
         multiSenseParameterSetter.setMultisenseResolution(rosMainNode);
         return multiSenseParameterSetter.setupNativeROSCommunicator(lidarSpindleVelocity);
      }

      return true;
   }

   private void registerCameraReceivers(Ros2Node ros2Node)
   {
      CameraLogger logger = LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
      cameraReceiver = new CameraDataReceiver(fullRobotModelFactory, cameraParameters.getPoseFrameForSdf(), robotConfigurationDataBuffer,
                                              new VideoPacketHandler(ros2Node), ppsTimestampOffsetProvider);

      new RosCameraCompressedImageReceiver(cameraParameters, rosMainNode, logger, cameraReceiver);

      cameraReceiver.start();
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      cameraReceiver.registerCameraListener(drcStereoListener);

   }
}
