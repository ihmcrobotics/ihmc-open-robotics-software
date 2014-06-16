package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import us.ihmc.darpaRoboticsChallenge.driving.DRCStereoListener;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.messages.controller.RobotPoseData;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.graphics3DAdapter.camera.CompressedVideoDataServer;
import us.ihmc.graphics3DAdapter.camera.CompressedVideoHandler;
import us.ihmc.graphics3DAdapter.camera.VideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.graphics3DAdapter.camera.VideoSettings.VideoCompressionKey;
import us.ihmc.utilities.ros.RosPoseStampedPublisher;

public abstract class CameraDataReceiver
{
   private final RobotPoseBuffer robotPoseBuffer;
   private final CompressedVideoDataServer compressedVideoDataServer; 
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();
   private final Vector3d cameraPositionVector = new Vector3d();

   private final Vector3d rootPosition = new Vector3d();
   private final Quat4d rootOrientation = new Quat4d();
   
   
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final Transform3D identityTransform = new Transform3D();
   private final Transform3D cameraPose;

   public CameraDataReceiver(RobotPoseBuffer robotPoseBuffer, VideoSettings videoSettings, final DRCNetworkProcessorNetworkingManager networkingManager,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.robotPoseBuffer = robotPoseBuffer;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.cameraPose = new Transform3D();

      compressedVideoDataServer = new CompressedVideoDataServer(videoSettings, new VideoPacketHandler(networkingManager));
      networkingManager.getControllerCommandHandler().setVideoCommandListener(compressedVideoDataServer);
   }
   
   protected void updateLeftEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      updateLeftEyeImage(null,null,identityTransform, bufferedImage, timeStamp, fov);
   }

   protected void updateLeftEyeImage(RosPoseStampedPublisher cameraPosePublisher,RosPoseStampedPublisher rootPosePublisher, Transform3D additionalTransformFromBaseToCamera, BufferedImage bufferedImage, long timeStamp, double fov)
   {
      for (int i = 0; i < stereoListeners.size(); i++)
      {
         stereoListeners.get(i).leftImage(bufferedImage, timeStamp, fov);
      }

      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timeStamp));
      if (robotPoseData == null)
      {
         return;
      }

      cameraPose.set(robotPoseData.getCameraPose());
      cameraPose.mul(additionalTransformFromBaseToCamera);
      cameraPose.get(cameraOrientation, cameraPositionVector);
      cameraPosition.set(cameraPositionVector);
      compressedVideoDataServer.updateImage(bufferedImage, timeStamp, cameraPosition, cameraOrientation, fov);
      
      if(cameraPosePublisher!=null && rootPosePublisher != null){
         robotPoseData.getPelvisPose().get(rootOrientation,rootPosition);
         rootPosePublisher.publish("/world", rootPosition, rootOrientation, Time.fromMillis(timeStamp));
         cameraPosePublisher.publish("/world", cameraPositionVector, cameraOrientation, Time.fromMillis(timeStamp));
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


   private class VideoPacketHandler implements CompressedVideoHandler
   {
      private final DRCNetworkProcessorNetworkingManager networkingManager;

      public VideoPacketHandler(DRCNetworkProcessorNetworkingManager networkingManager)
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
