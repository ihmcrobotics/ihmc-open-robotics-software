package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.communication.packets.sensing.VideoControlPacket;
import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.communication.producers.CompressedVideoDataServer;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.driving.DRCStereoListener;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public abstract class CameraDataReceiver
{
   private final RobotPoseBuffer robotPoseBuffer;
   private final CompressedVideoDataServer compressedVideoDataServer; 
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();
   private final Vector3d cameraPositionVector = new Vector3d();
   
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RigidBodyTransform cameraPose;

   public CameraDataReceiver(RobotPoseBuffer robotPoseBuffer, final AbstractNetworkProcessorNetworkingManager networkingManager,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.robotPoseBuffer = robotPoseBuffer;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.cameraPose = new RigidBodyTransform();

      compressedVideoDataServer = new CompressedVideoDataServer(new VideoPacketHandler(networkingManager));
      networkingManager.getControllerCommandHandler().attachListener(VideoControlPacket.class, compressedVideoDataServer);
   }
   
   protected void updateLeftEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timeStamp));
      if (robotPoseData == null)
      {
         return;
      }
      updateLeftEyeImage(robotPoseData.getCameraPoses()[0], bufferedImage, timeStamp, fov);
   }

   protected void updateLeftEyeImage(RigidBodyTransform worldToCamera, BufferedImage bufferedImage, long timeStamp, double fov)
   {
      cameraPose.set(worldToCamera);
      cameraPose.get(cameraOrientation, cameraPositionVector);
      cameraPosition.set(cameraPositionVector);
      updateLeftEyeImage(cameraPosition, cameraOrientation, bufferedImage, timeStamp, fov);
   }
   
   private void updateLeftEyeImage(Point3d position, Quat4d rotation, BufferedImage bufferedImage, long timeStamp, double fov)
   {
      for (int i = 0; i < stereoListeners.size(); i++)
      {
         stereoListeners.get(i).leftImage(bufferedImage, timeStamp, fov);
      }
      
      compressedVideoDataServer.updateImage(bufferedImage, timeStamp, position, rotation, fov);
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
      private final AbstractNetworkProcessorNetworkingManager networkingManager;

      public VideoPacketHandler(AbstractNetworkProcessorNetworkingManager networkingManager)
      {
         this.networkingManager = networkingManager;
      }

      public void newVideoPacketAvailable(long timeStamp, byte[] data, Point3d position, Quat4d orientation, double fieldOfView)
      {
         System.out.println("SENDING VIDEO PACKET");
         networkingManager.getControllerStateHandler().sendSerializableObject(
               new VideoPacket(timeStamp, data, position, orientation, fieldOfView));
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
