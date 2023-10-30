package us.ihmc.sensors;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.time.Instant;
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
   private final RestartableThrottledThread realsenseGrabThread;

   private int numberOfFailedReads = 0;

   public RealsenseColorDepthImageRetriever(RealsenseDeviceManager realsenseManager, String realsenseSerialNumber, RealsenseConfiguration realsenseConfiguration, Supplier<ReferenceFrame> sensorFrameSupplier)
   {
      this.sensorFrameSupplier = sensorFrameSupplier;
      this.realsenseManager = realsenseManager;
      this.realsenseSerialNumber = realsenseSerialNumber;
      this.realsenseConfiguration = realsenseConfiguration;

      realsenseGrabThread = new RestartableThrottledThread("RealsenseImageGrabber", OUTPUT_FREQUENCY, this::updateImages);
   }

   private void updateImages()
   {
      while (realsense == null ||realsense.getDevice() == null || numberOfFailedReads > 30)
      {
         startRealsense();
         ThreadTools.sleep(3000);
      }

      if (realsense.readFrameData())
      {
         realsense.updateDataBytePointers();
         Instant acquisitionTime = Instant.now();

         ReferenceFrame cameraFrame = sensorFrameSupplier.get();
         depthPose.setToZero(sensorFrameSupplier.get());
         depthPose.changeFrame(ReferenceFrame.getWorldFrame());

         colorPose.setIncludingFrame(cameraFrame, realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());
         colorPose.invert();
         colorPose.changeFrame(ReferenceFrame.getWorldFrame());

         if (depthMat16UC1 != null)
            depthMat16UC1.close();
         depthMat16UC1 = new Mat(realsense.getDepthHeight(), realsense.getDepthWidth(), opencv_core.CV_16UC1, realsense.getDepthFrameData());
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

         if (colorMatRGB != null)
            colorMatRGB.close();
         colorMatRGB = new Mat(realsense.getColorHeight(), realsense.getColorWidth(), opencv_core.CV_8UC3, realsense.getColorFrameData());
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

         grabSequenceNumber++;
      }
      else
      {
         numberOfFailedReads++;
      }
   }

   public RawImage getLatestRawDepthImage()
   {
      return depthImage;
   }

   public RawImage getLatestRawColorImage()
   {
      return colorImage;
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
      stop();
      depthImage.destroy();
      colorImage.destroy();
      realsense.deleteDevice();
      realsenseManager.deleteContext();
   }

   private boolean startRealsense()
   {
      LogTools.info("Starting Realsense...");
      if (realsense != null)
      {
         realsense.deleteDevice();
         realsenseManager.deleteContext();
      }

      realsenseManager = new RealsenseDeviceManager();
      realsense = realsenseManager.createBytedecoRealsenseDevice(realsenseSerialNumber, realsenseConfiguration);

      if (realsense != null)
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
