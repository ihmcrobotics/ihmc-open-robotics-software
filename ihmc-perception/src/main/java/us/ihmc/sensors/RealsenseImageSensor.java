package us.ihmc.sensors;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.function.Supplier;

public class RealsenseImageSensor extends ImageSensor
{
   public static final int COLOR_IMAGE_KEY = 0;
   public static final int DEPTH_IMAGE_KEY = 1;
   public static final int OUTPUT_IMAGE_COUNT = 2;

   private static final double OUTPUT_FREQUENCY = 20.0;

   private final RealsenseConfiguration realsenseConfiguration;
   private final RealsenseDeviceManager realsenseManager;
   private RealsenseDevice realsense = null;

   private final RawImage[] grabbedImages = new RawImage[OUTPUT_IMAGE_COUNT];
   private long grabSequenceNumber = 0L;
   private int grabFailureCount = 0;

   private final FramePose3D depthPose = new FramePose3D();
   private final FramePose3D colorPose = new FramePose3D();
   private final Supplier<ReferenceFrame> sensorFrameSupplier;
   private final Throttler grabThrottler = new Throttler().setFrequency(OUTPUT_FREQUENCY);

   public RealsenseImageSensor(RealsenseDeviceManager realsenseManager,
                               RealsenseConfiguration realsenseConfiguration,
                               Supplier<ReferenceFrame> sensorFrameSupplier)
   {
      this.sensorFrameSupplier = sensorFrameSupplier;
      this.realsenseManager = realsenseManager;
      this.realsenseConfiguration = realsenseConfiguration;
   }

   @Override
   protected boolean startSensor()
   {
      if (realsense != null)
      {
         if (realsense.getDevice() != null)
            realsense.deleteDevice();
         realsense = null;
      }

      realsense = realsenseManager.createBytedecoRealsenseDevice(realsenseConfiguration);

      boolean success = realsense != null && realsense.getDevice() != null;
      if (success)
      {
         LogTools.info("Initializing Realsense...");
         realsense.enableColor(realsenseConfiguration);
         realsense.initialize();
         grabFailureCount = 0;
      }
      else
      {
         LogTools.error("Failed to initialize Realsense");
      }

      return success;
   }

   @Override
   public boolean isSensorRunning()
   {
      return realsense != null && realsense.getDevice() != null && grabFailureCount < OUTPUT_FREQUENCY;
   }

   @Override
   protected boolean grab()
   {
      grabThrottler.waitAndRun();

      // Update the sensor poses
      ReferenceFrame sensorFrame = sensorFrameSupplier.get();
      depthPose.setToZero(sensorFrame);
      depthPose.changeFrame(ReferenceFrame.getWorldFrame());

      colorPose.setIncludingFrame(sensorFrame, realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());
      colorPose.invert();
      colorPose.changeFrame(ReferenceFrame.getWorldFrame());

      // Read grabbed images
      if (!realsense.readFrameData())
      {
         grabFailureCount++;
         return false;
      }
      Instant grabTime = Instant.now();
      grabSequenceNumber++;

      // Create mats with the images
      realsense.updateDataBytePointers();
      Mat bgrImage = new Mat(realsense.getColorHeight(), realsense.getColorWidth(), opencv_core.CV_8UC3, realsense.getColorFrameData());
      Mat depthImage = new Mat(realsense.getDepthHeight(), realsense.getDepthWidth(), opencv_core.CV_16UC1, realsense.getDepthFrameData());

      // Update grabbed images
      synchronized (grabbedImages)
      {
         grabbedImages[COLOR_IMAGE_KEY] = RawImage.createWithBGRImage(bgrImage, realsense.getColorCameraIntrinsics(), colorPose, grabTime, grabSequenceNumber);
         grabbedImages[DEPTH_IMAGE_KEY] = RawImage.createWith16BitDepth(depthImage,
                                                                        realsense.getDepthCameraIntrinsics(),
                                                                        depthPose,
                                                                        grabTime,
                                                                        grabSequenceNumber,
                                                                        (float) realsense.getDepthDiscretization());
      }

      grabFailureCount = 0;
      return true;
   }

   @Override
   public RawImage getImage(int imageKey)
   {
      synchronized (grabbedImages)
      {
         return new RawImage(grabbedImages[imageKey]);
      }
   }

   @Override
   public String getSensorName()
   {
      return realsenseConfiguration.name().split("_")[0];
   }

   @Override
   public void close()
   {
      System.out.println("Closing " + getClass().getSimpleName());
      super.close();

      // Release the images
      for (RawImage image : grabbedImages)
         if (image != null)
            image.release();

      // Close the camera
      if (realsense != null && realsense.getDevice() != null)
         realsense.deleteDevice();

      System.out.println("Closed " + getClass().getSimpleName());
   }
}
