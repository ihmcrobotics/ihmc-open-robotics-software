package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.driving.DRCStereoListener;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.messages.controller.RobotPoseData;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.graphics3DAdapter.camera.CompressedVideoDataServer;
import us.ihmc.graphics3DAdapter.camera.CompressedVideoHandler;
import us.ihmc.graphics3DAdapter.camera.VideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.graphics3DAdapter.camera.VideoSettings.VideoCompressionKey;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;

public abstract class CameraDataReceiver
{
   private final RobotPoseBuffer robotPoseBuffer;
   private final CompressedVideoDataServer multiSenseDataServer; 
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();
   private final Vector3d tempVector = new Vector3d();

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ROSNativeTransformTools rosTransformProvider;
   private Transform3D rosTransformFromHeadBaseToCamera = new Transform3D();

   private final Transform3D cameraPose;

   public CameraDataReceiver(RobotPoseBuffer robotPoseBuffer, VideoSettings videoSettings, final DRCNetworkProcessorNetworkingManager networkingManager,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.robotPoseBuffer = robotPoseBuffer;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.cameraPose = new Transform3D();

      if (DRCLocalConfigParameters.USE_ROS_FOR_MULTISENSE_TRANSFORMS)
      {
         rosTransformProvider = ROSNativeTransformTools.getInstance(DRCConfigParameters.ROS_MASTER_URI);
         rosTransformProvider.connect();
      }
      else
      {
         rosTransformProvider = null;
      }

      multiSenseDataServer = new CompressedVideoDataServer(videoSettings, new MultiSenseHandler(networkingManager));
      networkingManager.getControllerCommandHandler().setVideoCommandListener(multiSenseDataServer);


   }

   protected void updateLeftEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      for (int i = 0; i < stereoListeners.size(); i++)
      {
         stereoListeners.get(i).leftImage(bufferedImage, timeStamp, fov);
      }

      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.ajustTimeStampToRobotClock(timeStamp));
      if (robotPoseData == null)
      {
         return;
      }

      cameraPose.set(robotPoseData.getCameraPose());

      if (DRCLocalConfigParameters.USE_ROS_FOR_MULTISENSE_TRANSFORMS)
      {
         if ((rosTransformFromHeadBaseToCamera.getType() & Transform3D.IDENTITY) != 0)
         {
            getHeadToCameraTransform(timeStamp);
         }

         if ((rosTransformFromHeadBaseToCamera.getType() & Transform3D.ZERO) != 0)
         {
            rosTransformFromHeadBaseToCamera.setIdentity();
         }

         cameraPose.mul(rosTransformFromHeadBaseToCamera);
      }

      cameraPose.get(cameraOrientation, tempVector);
      cameraPosition.set(tempVector);
      multiSenseDataServer.updateImage(bufferedImage, timeStamp, cameraPosition, cameraOrientation, fov);
   }

   protected void getHeadToCameraTransform(long rosTimestamp)
   {
      if (ppsTimestampOffsetProvider.offsetIsDetermined())
      {
         long robotTimestamp = ppsTimestampOffsetProvider.ajustTimeStampToRobotClock(rosTimestamp);
         TimeStampedTransform3D transformFromRos = rosTransformProvider.getTimeStampedTransform("/left_camera_frame", "/head", rosTimestamp, robotTimestamp);
         rosTransformFromHeadBaseToCamera = transformFromRos.getTransform3D();
      }
   }

   protected void updateRightEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      for (int i = 0; i < stereoListeners.size(); i++)
      {
         stereoListeners.get(i).rightImage(bufferedImage, timeStamp, fov);
      }
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      stereoListeners.add(drcStereoListener);
   }


   private class MultiSenseHandler implements CompressedVideoHandler
   {
      private final DRCNetworkProcessorNetworkingManager networkingManager;

      public MultiSenseHandler(DRCNetworkProcessorNetworkingManager networkingManager)
      {
         this.networkingManager = networkingManager;
      }

      public void newVideoPacketAvailable(long timeStamp, byte[] data, Point3d position, Quat4d orientation, double fieldOfView,
            VideoCompressionKey videoCompressionKey)
      {
         networkingManager.getControllerStateHandler().sendVideoPacket(
               new VideoPacket(timeStamp, data, position, orientation, fieldOfView, videoCompressionKey));
      }

      public void addNetStateListener(CompressedVideoDataServer compressedVideoDataServer)
      {
         networkingManager.attachStateListener(compressedVideoDataServer);
      }

      public boolean isConnected()
      {
         return networkingManager.isConnected();
      }

     
   }

}
