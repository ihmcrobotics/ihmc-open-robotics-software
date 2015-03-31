package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.VideoDataServer;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;

public abstract class CameraDataReceiver extends Thread
{
   protected final boolean DEBUG = false;
   private final VideoDataServer compressedVideoDataServer;
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

   private final SDFFullRobotModel fullRobotModel;
   private ReferenceFrame cameraFrame;

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private final LinkedBlockingQueue<CameraData> dataQueue = new LinkedBlockingQueue<>();
   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private volatile boolean running = true;

   public CameraDataReceiver(SDFFullRobotModelFactory fullRobotModelFactory, String sensorNameInSdf, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         final PacketCommunicator sensorSuitePacketCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.cameraFrame = fullRobotModel.getCameraFrame(sensorNameInSdf);

      compressedVideoDataServer = CompressedVideoDataFactory.createCompressedVideoDataServer(sensorSuitePacketCommunicator, new VideoPacketHandler(sensorSuitePacketCommunicator));
   }

   public void setCameraFrame(ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;
   }

   public ReferenceFrame getHeadFrame()
   {
      return fullRobotModel.getHeadBaseFrame();
   }

   @Override
   public void run()
   {
      while (running)
      {
         try
         {
            CameraData data = dataQueue.take();
            if (data != null)
            {
               readWriteLock.writeLock().lock();

               if (DEBUG)
               {
                  System.out.println("Updating full robot model");
               }
               long robotTimestamp = ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(data.timestamp);
               if (robotConfigurationDataBuffer.updateFullRobotModel(false, robotTimestamp, fullRobotModel, null) < 0)
               {
                  continue;
               }
               cameraFrame.update();
               cameraFrame.getTransformToWorldFrame().get(cameraOrientation, cameraPosition);
               if (DEBUG)
               {
                  System.out.println(cameraFrame.getTransformToParent());
                  System.out.println(cameraPosition);
                  System.out.println(cameraOrientation);
               }
               for (int i = 0; i < stereoListeners.size(); i++)
               {
                  stereoListeners.get(i).newImageAvailable(data.robotSide, data.image, robotTimestamp, data.fov);
               }

               compressedVideoDataServer.updateImage(data.robotSide, data.image, robotTimestamp, cameraPosition, cameraOrientation, data.fov);
               readWriteLock.writeLock().unlock();
            }
         }
         catch (InterruptedException e)
         {
            continue;
         }
      }

   }

   protected void updateLeftEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      try
      {
         dataQueue.put(new CameraData(RobotSide.LEFT, bufferedImage, timeStamp, fov));
      }
      catch (InterruptedException e)
      {
      }
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      stereoListeners.add(drcStereoListener);
   }

   private static class CameraData
   {
      private final RobotSide robotSide;
      private final BufferedImage image;
      private final long timestamp;
      private final double fov;

      public CameraData(RobotSide robotSide, BufferedImage image, long timestamp, double fov)
      {
         this.robotSide = robotSide;
         this.image = image;
         this.timestamp = timestamp;
         this.fov = fov;
      }

   }

   private class VideoPacketHandler implements CompressedVideoHandler
   {
      private final PacketCommunicator packetCommunicator;

      public VideoPacketHandler(PacketCommunicator sensorSuitePacketCommunicator)
      {
         this.packetCommunicator = sensorSuitePacketCommunicator;
      }

      public void newVideoPacketAvailable(RobotSide robotSide, long timeStamp, byte[] data, Point3d position, Quat4d orientation, double fieldOfView)
      {
         if (DEBUG)
         {
            System.out.println(getClass().getName() + " sending new VideoPacket");
         }
         packetCommunicator.send(new VideoPacket(robotSide, timeStamp, data, position, orientation, fieldOfView));
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
