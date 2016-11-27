package us.ihmc.avatar.sensors.multisense;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.CameraLogger;
import us.ihmc.ihmcPerception.camera.RosCameraCompressedImageReceiver;
import us.ihmc.ihmcPerception.camera.VideoPacketHandler;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   public static boolean LOG_PRIMARY_CAMERA_IMAGES = false;

   private CameraDataReceiver cameraReceiver;

   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final RosMainNode rosMainNode;
   private final PacketCommunicator packetCommunicator;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private final DRCRobotCameraParameters cameraParameters;

   private MultiSenseParamaterSetter multiSenseParameterSetter;

   public MultiSenseSensorManager(FullHumanoidRobotModelFactory sdfFullRobotModelFactory, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         RosMainNode rosMainNode, PacketCommunicator sensorSuitePacketCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         DRCRobotCameraParameters cameraParameters, DRCRobotLidarParameters lidarParameters, DRCRobotPointCloudParameters stereoParameters,
         boolean setROSParameters)
   {
      this.fullRobotModelFactory = sdfFullRobotModelFactory;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.cameraParameters = cameraParameters;
      this.rosMainNode = rosMainNode;
      this.packetCommunicator = sensorSuitePacketCommunicator;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;

      registerCameraReceivers();
      if (setROSParameters)
      {
         multiSenseParameterSetter = new MultiSenseParamaterSetter(rosMainNode, sensorSuitePacketCommunicator);
         setMultiseSenseParams(lidarParameters.getLidarSpindleVelocity());
      }
      else
      {
         multiSenseParameterSetter = null;
      }
   }

   public void initializeParameterListeners()
   {
      System.out.println("initialise parameteres--------------------------------------------------------------------------------");
      if (multiSenseParameterSetter != null)
      {
         multiSenseParameterSetter.initializeParameterListeners();
      }
   }

   private void setMultiseSenseParams(double lidarSpindleVelocity)
   {
      if (multiSenseParameterSetter != null)
      {
         multiSenseParameterSetter.setMultisenseResolution(rosMainNode);
         multiSenseParameterSetter.setupNativeROSCommunicator(lidarSpindleVelocity);
      }
   }

   private void registerCameraReceivers()
   {
      CameraLogger logger = LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
      cameraReceiver = new CameraDataReceiver(fullRobotModelFactory, cameraParameters.getPoseFrameForSdf(), robotConfigurationDataBuffer, new VideoPacketHandler(packetCommunicator),
            ppsTimestampOffsetProvider);

      new RosCameraCompressedImageReceiver(cameraParameters, rosMainNode, logger, cameraReceiver);

      cameraReceiver.start();
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      cameraReceiver.registerCameraListener(drcStereoListener);

   }
}
