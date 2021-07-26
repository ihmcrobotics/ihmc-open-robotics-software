package us.ihmc.avatar.sensors.multisense;

import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.communication.producers.VideoControlSettings;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.ihmcPerception.camera.CameraLogger;
import us.ihmc.ihmcPerception.camera.RosCameraCompressedImageReceiver;
import us.ihmc.ihmcPerception.camera.VideoPacketHandler;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.utilities.ros.RosMainNode;

public class MultiSenseSensorManager
{
   public static boolean LOG_PRIMARY_CAMERA_IMAGES = false;

   private final RosMainNode rosMainNode;
   private final ROS2NodeInterface ros2Node;
   private final AvatarRobotCameraParameters cameraParameters;
   private final AvatarRobotLidarParameters lidarParameters;
   private boolean setROSParameters;

   private CameraDataReceiver cameraReceiver;
   private MultiSenseParamaterSetter multiSenseParameterSetter;
   private RosCameraCompressedImageReceiver cameraImageReceiver;
   private VideoPacketHandler compressedVideoHandler;

   public MultiSenseSensorManager(FullRobotModelFactory fullRobotModelFactory, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
                                  RosMainNode rosMainNode, ROS2NodeInterface ros2Node, RobotROSClockCalculator rosClockCalculator,
                                  AvatarRobotCameraParameters cameraParameters, AvatarRobotLidarParameters lidarParameters,
                                  AvatarRobotPointCloudParameters stereoParameters, boolean setROSParameters)
   {
      this.ros2Node = ros2Node;
      this.cameraParameters = cameraParameters;
      this.rosMainNode = rosMainNode;
      this.lidarParameters = lidarParameters;
      this.setROSParameters = setROSParameters;

      compressedVideoHandler = new VideoPacketHandler(ros2Node);
      cameraReceiver = new CameraDataReceiver(fullRobotModelFactory,
                                              cameraParameters.getPoseFrameForSdf(),
                                              robotConfigurationDataBuffer,
                                              compressedVideoHandler,
                                              rosClockCalculator::computeRobotMonotonicTime);
   }

   private boolean initializeParameterListenersRequested = false;

   public void initializeParameterListeners()
   {
      if (multiSenseParameterSetter != null)
      {
         multiSenseParameterSetter.initializeParameterListeners();
         initializeParameterListenersRequested = false;
      }
      else
      {
         initializeParameterListenersRequested = true;
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

   public void start()
   {
      boolean rosOnline = false;

      while (!rosOnline)
      {
         CameraLogger logger = LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
         cameraImageReceiver = new RosCameraCompressedImageReceiver(cameraParameters, rosMainNode, logger, cameraReceiver);

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

         if (rosOnline && initializeParameterListenersRequested)
            initializeParameterListeners();

         try
         {
            if (!rosOnline)
            {
               LogTools.info("Wainting for ROS to come online.");
               Thread.sleep(500);
            }
         }
         catch (InterruptedException e)
         {
            closeAndDispose();
            return;
         }
      }

      cameraReceiver.start();
   }

   public void setVideoSettings(VideoControlSettings settings)
   {
      cameraReceiver.setVideoSettings(settings);
   }

   public void closeAndDispose()
   {
      if (compressedVideoHandler != null)
         compressedVideoHandler.closeAndDispose();
      if (cameraReceiver != null)
         cameraReceiver.closeAndDispose();
      if (cameraImageReceiver != null)
         cameraImageReceiver.closeAndDispose();
   }
}
