package us.ihmc.sensors.realsense;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.ImageSensor;

import java.time.Instant;

public class RealSenseImageSensor extends ImageSensor
{
   public static final int COLOR_IMAGE_KEY = 0;
   public static final int DEPTH_IMAGE_KEY = 1;
   public static final int OUTPUT_IMAGE_COUNT = 2;

   protected final RealSenseConfiguration realsenseConfiguration;
   private final RealSenseDeviceManager realsenseManager = new RealSenseDeviceManager();
   protected RealSenseDevice realsense = null;

   private final RawImage[] grabbedImages = new RawImage[OUTPUT_IMAGE_COUNT];
   private long grabSequenceNumber = 0L;
   protected int grabFailureCount = 0;

   private ReferenceFrame depthFrame;
   private ReferenceFrame colorFrame;
   private final Notification resetFrames = new Notification();

   private final Throttler grabThrottler;

   public RealSenseImageSensor(RealSenseConfiguration realsenseConfiguration)
   {
      super(realsenseConfiguration.name().split("_")[0]);

      this.realsenseConfiguration = realsenseConfiguration;

      grabThrottler = new Throttler().setFrequency(realsenseConfiguration.getDepthFPS());
      resetFrames.set();
   }

   @Override
   protected void onSensorFrameChanged()
   {
      resetFrames.set();
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

   private boolean caughtAnException = false;

   @Override
   public boolean isSensorRunning()
   {
      return realsense != null && realsense.getDevice() != null && grabFailureCount < realsenseConfiguration.getDepthFPS();
   }

   @Override
   protected boolean grab()
   {
      grabThrottler.waitAndRun();

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

      // Update sensor frames
      if (resetFrames.poll())
      {
         if (depthFrame != null)
            depthFrame.remove();
         depthFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(getSensorName() + "_depth", sensorFrame, new RigidBodyTransform());

         if (colorFrame != null)
            colorFrame.remove();
         RigidBodyTransform colorToDepthTransform = new RigidBodyTransform(realsense.getDepthToColorRotation(), realsense.getDepthToColorTranslation());
         colorToDepthTransform.invert();
         colorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(getSensorName() + "_color", sensorFrame, colorToDepthTransform);
      }

      // Update grabbed images
      synchronized (grabbedImages)
      {
         try
         {
            if (grabbedImages[COLOR_IMAGE_KEY] != null)
               grabbedImages[COLOR_IMAGE_KEY].release();
            grabbedImages[COLOR_IMAGE_KEY] = RawImage.createWithBGRImage(bgrImage,
                                                                         realsense.getColorCameraIntrinsics(),
                                                                         colorFrame.getTransformToRoot(),
                                                                         grabTime,
                                                                         grabSequenceNumber);

            if (grabbedImages[DEPTH_IMAGE_KEY] != null)
               grabbedImages[DEPTH_IMAGE_KEY].release();
            grabbedImages[DEPTH_IMAGE_KEY] = RawImage.createWith16BitDepth(depthImage,
                                                                           realsense.getDepthCameraIntrinsics(),
                                                                           depthFrame.getTransformToRoot(),
                                                                           grabTime,
                                                                           grabSequenceNumber,
                                                                           (float) realsense.getDepthDiscretization());
         }
         catch (NotARotationMatrixException e)
         {
            if (!caughtAnException)
               LogTools.error("Caught a not a rotation. Only printing once.\n " + e.getMessage());
            caughtAnException = true;
         }
      }

      grabFailureCount = 0;
      return true;
   }

   @Override
   public int[] getImageKeys()
   {
      return new int[] {COLOR_IMAGE_KEY, DEPTH_IMAGE_KEY};
   }

   @Override
   public RawImage getImage(int imageKey)
   {
      synchronized (grabbedImages)
      {
         if (grabbedImages[imageKey] == null)
            return null;

         return grabbedImages[imageKey].get();
      }
   }

   @Override
   public ReferenceFrame getImageFrame(int imageKey)
   {
      return switch (imageKey)
      {
         case COLOR_IMAGE_KEY -> colorFrame;
         case DEPTH_IMAGE_KEY -> depthFrame;
         default -> null;
      };
   }

   @Override
   public ReferenceFrame[] getImageFrames()
   {
      return new ReferenceFrame[] {colorFrame, depthFrame};
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
      if (realsense != null)
      {
         realsense.deleteDevice();
         realsense = null;
      }

      // Don't delete context here - it can cause crashes with background threads
      // The context will be cleaned up by the JVM when the application exits
      // realsenseManager.deleteContext();

      System.out.println("Closed " + getClass().getSimpleName());
   }
}
