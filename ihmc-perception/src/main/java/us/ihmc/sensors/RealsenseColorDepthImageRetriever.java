package us.ihmc.sensors;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.time.Instant;
import java.util.function.Supplier;

public class RealsenseColorDepthImageRetriever
{
   private static final double OUTPUT_FREQUENCY = 20.0;

   private final BytedecoRealsense realsense;

   private long grabSequenceNumber = 0L;

   private RawImage depthImage = null;
   private RawImage colorImage = null;

   private Mat depthMat16UC1;
   private Mat colorMatRGB;

   private final FramePose3D cameraFramePose = new FramePose3D();
   private final FramePose3D colorPoseInDepthFrame = new FramePose3D();
   private final Supplier<ReferenceFrame> sensorFrameSupplier;
   private final RestartableThrottledThread realsenseGrabThread;

   public RealsenseColorDepthImageRetriever(BytedecoRealsense realsense, RealsenseConfiguration realsenseConfiguration, Supplier<ReferenceFrame> sensorFrameSupplier)
   {
      this.sensorFrameSupplier = sensorFrameSupplier;

      this.realsense = realsense;

      if (realsense.getDevice() == null)
      {
         // Find something else to do here
         System.exit(1);
      }
      realsense.enableColor(realsenseConfiguration);
      realsense.initialize();

      realsenseGrabThread = new RestartableThrottledThread("RealsenseImageGrabber", OUTPUT_FREQUENCY, this::updateImages);
   }

   private void updateImages()
   {
      if (realsense.readFrameData())
      {
         realsense.updateDataBytePointers();
         Instant acquisitionTime = Instant.now();

         cameraFramePose.setToZero(sensorFrameSupplier.get());
         cameraFramePose.changeFrame(ReferenceFrame.getWorldFrame());

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
                                   cameraFramePose.getPosition(),
                                   cameraFramePose.getOrientation());

         colorPoseInDepthFrame.set(realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());

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
                                   colorPoseInDepthFrame.getPosition(),
                                   colorPoseInDepthFrame.getOrientation());

         grabSequenceNumber++;
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
   }
}
