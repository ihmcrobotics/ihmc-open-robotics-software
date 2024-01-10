package us.ihmc.perception.opencv;

import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.tools.thread.Throttler;

public class OpenCVArUcoMarkerDetectionThread
{
   private SwapReference<OpenCVArUcoMakerDetectionSwapData> detectionSwapReference;
   private DetectorParameters detectorParametersForTuners;
   private boolean enabled = true;
   private volatile boolean running = true;
   private final Timer timer = new Timer();
   private final Throttler throttler = new Throttler().setFrequency(20.0);
   private final OpenCVArUcoMarkerDetection detection = new OpenCVArUcoMarkerDetection();
   private final OpenCVArUcoMarkerDetectionOutput output = new OpenCVArUcoMarkerDetectionOutput();
   private BytedecoImage optionalSourceColorImage;

   public OpenCVArUcoMarkerDetectionThread()
   {
      detectionSwapReference = new SwapReference<>(OpenCVArUcoMakerDetectionSwapData::new);
      detectorParametersForTuners = new DetectorParameters();

      ThreadTools.startAsDaemon(this::detectMarkersOnAsynchronousThread, "ArUcoMarkerDetection");
   }

   public void setSourceImageForDetection(BytedecoImage optionalSourceColorImage)
   {
      this.optionalSourceColorImage = optionalSourceColorImage;
   }

   public void update()
   {
      update(optionalSourceColorImage);
   }

   public void update(BytedecoImage sourceColorImage)
   {
      if (enabled)
      {
         timer.reset();

         synchronized (detectionSwapReference)
         {
            OpenCVArUcoMakerDetectionSwapData data = detectionSwapReference.getForThreadTwo();
            data.ensureImageInitialized(sourceColorImage.getImageWidth(), sourceColorImage.getImageHeight());
            OpenCVArUcoMarkerDetection.convertOrCopyToRGB(sourceColorImage, data.getRgb8ImageForDetection());
            OpenCVArUcoMarkerDetectionParametersTools.copy(detectorParametersForTuners, detection.getDetectorParameters());
         }
      }
   }

   private void detectMarkersOnAsynchronousThread()
   {
      while (running)
      {
         throttler.waitAndRun();
         if (timer.isRunning(0.5))
         {
            OpenCVArUcoMakerDetectionSwapData data = detectionSwapReference.getForThreadTwo();

            if (data.getRgb8ImageForDetection() != null)
            {
               detection.performDetection(data.getRgb8ImageForDetection());
               data.getOutput().set(detection);
               data.setHasDetected(true);
            }

            detectionSwapReference.swap();
         }
      }
   }

   public void destroy()
   {
      System.out.println("Destroying aruco marker detection");
      running = false;
   }

   /**
    * Providing this because the classes that use this need different data at different times, which is not thread safe anymore,
    * so you can use this to make sure all the stuff you get from the getters is from the same detection result. To use, do
    * <pre>
    *    synchronized (arUcoMarkerDetection.getSyncObject())
    *    {
    *       arUcoMarkerDetection.getIDsMat()
    *       arUcoMarkerDetection.updateMarkerPose(...)
    *       ...
    *    }
    * </pre>
    */
   public Object getSyncObject()
   {
      return detectionSwapReference;
   }

   public OpenCVArUcoMarkerDetection getDetection()
   {
      return detection;
   }

   public OpenCVArUcoMarkerDetectionOutput getOutput()
   {
      return output;
   }

   public DetectorParameters getDetectorParameters()
   {
      return detectorParametersForTuners;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public boolean isEnabled()
   {
      return enabled;
   }
}
