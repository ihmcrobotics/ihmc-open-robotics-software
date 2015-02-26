package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.VideoDataServer;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;

public abstract class CameraDataReceiver
{
   protected final boolean DEBUG = false;
   private final VideoDataServer compressedVideoDataServer;
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

   private final SDFFullRobotModel fullRobotModel;
   private ReferenceFrame cameraFrame;

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();
   private final Vector3d cameraPositionVector = new Vector3d();

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   public CameraDataReceiver(SDFFullRobotModelFactory fullRobotModelFactory, String sensorNameInSdf, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         final PacketCommunicator packetCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.cameraFrame = fullRobotModel.getCameraFrame(sensorNameInSdf);

      compressedVideoDataServer = CompressedVideoDataFactory.createCompressedVideoDataServer(packetCommunicator, new VideoPacketHandler(packetCommunicator));
   }

   public void setCameraFrame(ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;
   }

   public ReferenceFrame getHeadFrame()
   {
      return fullRobotModel.getHeadBaseFrame();
   }

   protected void updateLeftEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      if(DEBUG)
      {
         System.out.println("Updating full robot model");
      }
      if (robotConfigurationDataBuffer.updateFullRobotModel(false, ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timeStamp), fullRobotModel, null) < 0)
      {
         return;
      }
      cameraFrame.getTransformToWorldFrame().get(cameraOrientation, cameraPositionVector);
      if(DEBUG)
      {
         
         System.out.println(cameraPositionVector);
         System.out.println(cameraOrientation);
      }
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
         if (DEBUG)
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
