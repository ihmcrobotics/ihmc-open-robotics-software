package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.VideoDataServer;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;

public abstract class CameraDataReceiver
{
   private final boolean DEBUG = false;
   private final RobotPoseBuffer robotPoseBuffer;
   private final VideoDataServer compressedVideoDataServer;
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();
   private final Vector3d cameraPositionVector = new Vector3d();

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RigidBodyTransform cameraPose;

   public CameraDataReceiver(RobotPoseBuffer robotPoseBuffer, final PacketCommunicator packetCommunicator,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.robotPoseBuffer = robotPoseBuffer;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.cameraPose = new RigidBodyTransform();

      compressedVideoDataServer = CompressedVideoDataFactory.createCompressedVideoDataServer(packetCommunicator, new VideoPacketHandler(packetCommunicator));
   }

   protected void updateLeftEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timeStamp));
      if (robotPoseData == null)
      {
         if(DEBUG)
         {
            System.out.println(getClass().getName() + " robot pose data was null! Camera image wont be updated");
         }
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
      private final PacketCommunicator packetCommunicator;

      public VideoPacketHandler(PacketCommunicator packetCommunicator)
      {
         this.packetCommunicator = packetCommunicator;
      }

      public void newVideoPacketAvailable(long timeStamp, byte[] data, Point3d position, Quat4d orientation, double fieldOfView)
      {
         if(DEBUG)
         {
            System.out.println(getClass().getName() + " sending new VideoPacket");
         }
         packetCommunicator.send(new VideoPacket(timeStamp, data, position, orientation, fieldOfView));
      }

      public void addNetStateListener(NetStateListener compressedVideoDataServer)
      {
         packetCommunicator.attachStateListener(compressedVideoDataServer);
      }

      public boolean isConnected()
      {
         return packetCommunicator.isConnected();
      }

   }

}
