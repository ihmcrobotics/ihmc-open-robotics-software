package us.ihmc.perception.ros1.camera;

import java.awt.image.BufferedImage;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.LongUnaryOperator;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.CompressedVideoDataServer;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoControlSettings;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.sensorData.CameraData;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class CameraDataReceiver implements CloseableAndDisposable
{
   private static final boolean DEBUG = false;
   private final CompressedVideoDataServer compressedVideoDataServer;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;

   private final FullRobotModel fullRobotModel;
   private ReferenceFrame cameraFrame;

   private final Point3D cameraPosition = new Point3D();
   private final Quaternion cameraOrientation = new Quaternion();

   private final LongUnaryOperator robotMonotonicTimeCalculator;

   private final LinkedBlockingQueue<CameraData> dataQueue = new LinkedBlockingQueue<>(2);
   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();

   private boolean useTimestamps = true;
   private long initialDelayMilliseconds = 1000L;
   private long publishingPeriodMilliseconds = 200L;

   private final AtomicReference<VideoControlSettings> newVideoSettings = new AtomicReference<>(null);

   private final ScheduledExecutorService executorService = ThreadTools.newSingleDaemonThreadScheduledExecutor(getClass().getName());

   private boolean hasStarted = false;
   private boolean isPaused = false;

   public CameraDataReceiver(FullRobotModelFactory fullRobotModelFactory,
                             String sensorNameInSdf,
                             RobotConfigurationDataBuffer robotConfigurationDataBuffer,
                             CompressedVideoHandler compressedVideoHandler,
                             LongUnaryOperator robotMonotonicTimeCalculator)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      this.robotMonotonicTimeCalculator = robotMonotonicTimeCalculator;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      cameraFrame = fullRobotModel.getCameraFrame(sensorNameInSdf);

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

   public void setInitialDelayMilliseconds(long initialDelayMilliseconds)
   {
      this.initialDelayMilliseconds = initialDelayMilliseconds;
   }

   public void setUseTimestamps(boolean useTimestamps)
   {
      this.useTimestamps = useTimestamps;
   }

   private ScheduledFuture<?> activeTask;

   public void start()
   {
      hasStarted = true;
      if (!isPaused)
         startThread();
   }

   public void pause()
   {
      if (isPaused)
         return;

      if (!hasStarted)
      {
         isPaused = true;
         return;
      }

      activeTask.cancel(false);
      activeTask = null;
      isPaused = true;
   }

   public void resume()
   {
      if (!isPaused)
         return;

      if (!hasStarted)
      {
         isPaused = false;
         return;
      }

      startThread();
      isPaused = false;
   }

   private void restart()
   {
      if (!hasStarted || isPaused)
         return;

      activeTask.cancel(false);
      startThread();
   }

   private void startThread()
   {
      activeTask = executorService.scheduleAtFixedRate(this::run, initialDelayMilliseconds, publishingPeriodMilliseconds, TimeUnit.MILLISECONDS);
   }

   private void run()
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

            if (useTimestamps)
            {
               if (robotConfigurationDataBuffer.updateFullRobotModel(false, robotTimestamp, fullRobotModel, null) < 0)
               {
                  if (DEBUG)
                  {
                     System.out.println("Cannot update full robot model, skipping frame");
                  }

                  return;
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

            VideoControlSettings settings = newVideoSettings.getAndSet(null);

            if (settings != null)
               compressedVideoDataServer.setVideoControlSettings(settings);

            compressedVideoDataServer.onFrame(data.videoSource, data.image, robotTimestamp, cameraPosition, cameraOrientation, data.intrinsicParameters);
            readWriteLock.writeLock().unlock();
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public void updateImage(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, CameraPinholeBrown intrinsicParameters)
   {
      dataQueue.offer(new CameraData(videoSource, bufferedImage, timeStamp, intrinsicParameters));
   }

   public void setVideoSettings(VideoControlSettings settings)
   {
      this.newVideoSettings.set(settings);

      if (!settings.isSendVideo())
      {
         pause();
         return;
      }

      int fps = settings.getFPS();
      long periodInMilliseconds = (long) Conversions.secondsToMilliseconds(1.0 / (double) fps);

      if (periodInMilliseconds != publishingPeriodMilliseconds)
      {
         publishingPeriodMilliseconds = periodInMilliseconds;

         restart();
      }
      else
      {
         resume();
      }
   }

   @Override
   public void closeAndDispose()
   {
      executorService.shutdownNow();
   }
}
