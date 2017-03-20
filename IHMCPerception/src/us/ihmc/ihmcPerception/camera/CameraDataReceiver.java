package us.ihmc.ihmcPerception.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.sensorData.CameraData;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;

public class CameraDataReceiver extends Thread
{
   private static final boolean DEBUG = false;
   private final VideoDataServer compressedVideoDataServer;
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

   private final FullRobotModel fullRobotModel;
   private ReferenceFrame cameraFrame;

   private final Point3D cameraPosition = new Point3D();
   private final Quaternion cameraOrientation = new Quaternion();

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private final LinkedBlockingQueue<CameraData> dataQueue = new LinkedBlockingQueue<>(2);
   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private volatile boolean running = true;

   private boolean useTimestamps = true;

   public CameraDataReceiver(FullRobotModelFactory fullRobotModelFactory, String sensorNameInSdf, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         CompressedVideoHandler compressedVideoHandler, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.cameraFrame = fullRobotModel.getCameraFrame(sensorNameInSdf);

      compressedVideoDataServer = CompressedVideoDataFactory.createCompressedVideoDataServer(compressedVideoHandler);
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

               if(useTimestamps)
               {
                  if (robotConfigurationDataBuffer.updateFullRobotModel(false, robotTimestamp, fullRobotModel, null) < 0)
                  {
                     if(DEBUG)
                     {
                        System.out.println("Cannot update full robot model, skipping frame");
                     }

                     continue;
                  }
               }
               else
               {
                  robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
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
                  stereoListeners.get(i).newImageAvailable(data, cameraFrame.getTransformToWorldFrame());
               }

               compressedVideoDataServer.updateImage(data.videoSource, data.image, robotTimestamp, cameraPosition, cameraOrientation, data.intrinsicParameters);
               readWriteLock.writeLock().unlock();
            }
         }
         catch (InterruptedException e)
         {
            continue;
         }
      }
   }

   public void setUseTimestamps(boolean useTimestamps)
   {
      this.useTimestamps = useTimestamps;
   }

   public void updateImage(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, IntrinsicParameters intrinsicParameters)
   {
      dataQueue.offer(new CameraData(videoSource, bufferedImage, timeStamp, intrinsicParameters));
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      stereoListeners.add(drcStereoListener);
   }
}
