package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.sensorProcessing.sensorData.CameraData;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;

public abstract class CameraDataReceiver extends Thread
{
   protected static final boolean DEBUG = false;
   private final VideoDataServer compressedVideoDataServer;
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

   private final SDFFullRobotModel fullRobotModel;
   private ReferenceFrame cameraFrame;

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   private final LinkedBlockingQueue<CameraData> dataQueue = new LinkedBlockingQueue<>(2);
   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private volatile boolean running = true;
   
   public CameraDataReceiver(SDFFullRobotModelFactory fullRobotModelFactory, String sensorNameInSdf, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
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
                  stereoListeners.get(i).newImageAvailable(data, cameraFrame.getTransformToWorldFrame());
               }
               
               compressedVideoDataServer.updateImage(data.robotSide, data.image, robotTimestamp, cameraPosition, cameraOrientation, data.intrinsicParameters);
               readWriteLock.writeLock().unlock();
            }
         }
         catch (InterruptedException e)
         {
            continue;
         }
      }
   }
   
   protected void updateImage(RobotSide robotSide, BufferedImage bufferedImage, long timeStamp, IntrinsicParameters intrinsicParameters)
   {
      dataQueue.offer(new CameraData(robotSide, bufferedImage, timeStamp, intrinsicParameters));
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      stereoListeners.add(drcStereoListener);
   }
}
