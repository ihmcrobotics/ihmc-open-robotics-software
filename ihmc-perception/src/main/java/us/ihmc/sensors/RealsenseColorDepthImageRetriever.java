package us.ihmc.sensors;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.time.Instant;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

public class RealsenseColorDepthImageRetriever
{
   private static final double OUTPUT_FREQUENCY = 20.0;

   private final String realsenseSerialNumber;
   private final RealsenseConfiguration realsenseConfiguration;
   private RealsenseDeviceManager realsenseManager;
   private RealsenseDevice realsense = null;

   private long grabSequenceNumber = 0L;

   private RawImage depthImage = null;
   private RawImage colorImage = null;

   private Mat depthMat16UC1;
   private Mat colorMatRGB;

   private final FramePose3D depthPose = new FramePose3D();
   private final FramePose3D colorPose = new FramePose3D();
   private final Supplier<ReferenceFrame> sensorFrameSupplier;
   private final ROS2DemandGraphNode demandGraphNode;
   private final RestartableThrottledThread realsenseGrabThread;

   private final Lock newDepthImageLock = new ReentrantLock();
   private final Condition newDepthImageAvailable = newDepthImageLock.newCondition();
   private long lastDepthSequenceNumber = -1L;

   private final Lock newColorImageLock = new ReentrantLock();
   private final Condition newColorImageAvailable = newColorImageLock.newCondition();
   private long lastColorSequenceNumber = -1L;

   private int numberOfFailedReads = 0;

   public RealsenseColorDepthImageRetriever(RealsenseDeviceManager realsenseManager,
                                            String realsenseSerialNumber,
                                            RealsenseConfiguration realsenseConfiguration,
                                            Supplier<ReferenceFrame> sensorFrameSupplier,
                                            ROS2DemandGraphNode realsenseDemandNode)
   {
      this.sensorFrameSupplier = sensorFrameSupplier;
      this.realsenseManager = realsenseManager;
      this.realsenseSerialNumber = realsenseSerialNumber;
      this.realsenseConfiguration = realsenseConfiguration;
      this.demandGraphNode = realsenseDemandNode;

      realsenseGrabThread = new RestartableThrottledThread("RealsenseImageGrabber", OUTPUT_FREQUENCY, this::updateImages);
      realsenseGrabThread.start();
   }

   private void updateImages()
   {
      if (demandGraphNode.isDemanded())
      {
         if (realsense == null || realsense.getDevice() == null || numberOfFailedReads > 30)
         {
            if (!startRealsense())
               ThreadTools.sleep(3000);
         }
         else if (realsense.readFrameData())
         {
            realsense.updateDataBytePointers();
            Instant acquisitionTime = Instant.now();

            numberOfFailedReads = 0;

            ReferenceFrame cameraFrame = sensorFrameSupplier.get();
            depthPose.setToZero(sensorFrameSupplier.get());
            depthPose.changeFrame(ReferenceFrame.getWorldFrame());

            colorPose.setIncludingFrame(cameraFrame, realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());
            colorPose.invert();
            colorPose.changeFrame(ReferenceFrame.getWorldFrame());

            if (depthMat16UC1 != null)
               depthMat16UC1.close();
            depthMat16UC1 = new Mat(realsense.getDepthHeight(), realsense.getDepthWidth(), opencv_core.CV_16UC1, realsense.getDepthFrameData());

            newDepthImageLock.lock();
            try
            {
               if (depthImage != null)
                  depthImage.release();
               depthImage = new RawImage(grabSequenceNumber,
                                         acquisitionTime,
                                         realsense.getDepthWidth(),
                                         realsense.getDepthHeight(),
                                         (float) realsense.getDepthDiscretization(),
                                         depthMat16UC1.clone(),
                                         null,
                                         opencv_core.CV_16UC1,
                                         (float) realsense.getDepthFocalLengthPixelsX(),
                                         (float) realsense.getDepthFocalLengthPixelsY(),
                                         (float) realsense.getDepthPrincipalOffsetXPixels(),
                                         (float) realsense.getDepthPrincipalOffsetYPixels(),
                                         depthPose.getPosition(),
                                         depthPose.getOrientation());

               newDepthImageAvailable.signal();
            }
            finally
            {
               newDepthImageLock.unlock();
            }

            if (colorMatRGB != null)
               colorMatRGB.close();
            colorMatRGB = new Mat(realsense.getColorHeight(), realsense.getColorWidth(), opencv_core.CV_8UC3, realsense.getColorFrameData());

            newColorImageLock.lock();
            try
            {
               if (colorImage != null)
                  colorImage.release();
               colorImage = new RawImage(grabSequenceNumber,
                                         acquisitionTime,
                                         realsense.getColorWidth(),
                                         realsense.getColorHeight(),
                                         (float) realsense.getDepthDiscretization(),
                                         colorMatRGB.clone(),
                                         null,
                                         opencv_core.CV_8UC3,
                                         (float) realsense.getColorFocalLengthPixelsX(),
                                         (float) realsense.getColorFocalLengthPixelsY(),
                                         (float) realsense.getColorPrincipalOffsetXPixels(),
                                         (float) realsense.getColorPrincipalOffsetYPixels(),
                                         colorPose.getPosition(),
                                         colorPose.getOrientation());

               newColorImageAvailable.signal();
            }
            finally
            {
               newColorImageLock.unlock();
            }

            grabSequenceNumber++;
         }
         else
         {
            numberOfFailedReads++;
         }
      }
      else
         ThreadTools.sleep(500);
   }

   public RawImage getLatestRawDepthImage()
   {
      newDepthImageLock.lock();
      try
      {
         while (depthImage == null || depthImage.isEmpty() || depthImage.getSequenceNumber() == lastDepthSequenceNumber)
         {
            newDepthImageAvailable.await();
         }

         lastDepthSequenceNumber = depthImage.getSequenceNumber();
      }
      catch (InterruptedException interruptedException)
      {
         LogTools.error(interruptedException.getMessage());
      }

      return depthImage.get();
   }

   public RawImage getLatestRawColorImage()
   {
      newColorImageLock.lock();
      try
      {
         while (colorImage == null || colorImage.isEmpty() || colorImage.getSequenceNumber() == lastColorSequenceNumber)
         {
            newColorImageAvailable.await();
         }

         lastColorSequenceNumber = colorImage.getSequenceNumber();
      }
      catch (InterruptedException interruptedException)
      {
         LogTools.error(interruptedException.getMessage());
      }

      return colorImage.get();
   }

   public void start()
   {
      realsenseGrabThread.start();
   }

   public void stop()
   {
      realsenseGrabThread.stop();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      stop();

      if (depthImage != null)
         depthImage.release();
      if (colorImage != null)
         colorImage.release();
      if (realsense != null && realsense.getDevice() != null)
         realsense.deleteDevice();
      realsenseManager.deleteContext();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private boolean startRealsense()
   {
      LogTools.info("Starting Realsense...");
      if (realsense != null)
      {
         if (realsense.getDevice() != null)
            realsense.deleteDevice();
         realsenseManager.deleteContext();
      }

      realsenseManager = new RealsenseDeviceManager();
      realsense = realsenseManager.createBytedecoRealsenseDevice(realsenseSerialNumber, realsenseConfiguration);

      if (realsense != null && realsense.getDevice() != null)
      {
         LogTools.info("Initializing Realsense...");
         realsense.enableColor(realsenseConfiguration);
         realsense.initialize();

         numberOfFailedReads = 0;
      }
      else
      {
         LogTools.error("Failed to initialize Realsense");
      }

      return realsense != null;
   }
}
