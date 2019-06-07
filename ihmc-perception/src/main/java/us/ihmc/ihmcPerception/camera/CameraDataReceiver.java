package us.ihmc.ihmcPerception.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.LongUnaryOperator;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
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

   private final LongUnaryOperator robotMonotonicTimeCalculator;

   private final LinkedBlockingQueue<CameraData> dataQueue = new LinkedBlockingQueue<>(2);
   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private volatile boolean running = true;

   private boolean useTimestamps = true;

   public CameraDataReceiver(FullRobotModelFactory fullRobotModelFactory, String sensorNameInSdf, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         CompressedVideoHandler compressedVideoHandler, LongUnaryOperator robotMonotonicTimeCalculator)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      this.robotMonotonicTimeCalculator = robotMonotonicTimeCalculator;
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
               long robotTimestamp = robotMonotonicTimeCalculator.applyAsLong(data.timestamp);

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

               compressedVideoDataServer.onFrame(data.videoSource, data.image, robotTimestamp, cameraPosition, cameraOrientation, data.intrinsicParameters);
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
