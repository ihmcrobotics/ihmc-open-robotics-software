package us.ihmc.perception.opencv;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.tools.thread.Throttler;

public class OpenCVArUcoMarkerDetectionThread
{
   private SwapReference<OpenCVArUcoMakerDetectionSwapData> detectionSwapReference;
   private boolean enabled = true;
   private volatile boolean running = true;
   private final Timer timer = new Timer();
   private final Throttler throttler = new Throttler().setFrequency(20.0);

   public OpenCVArUcoMarkerDetectionThread()
   {
      detectionSwapReference = new SwapReference<>(OpenCVArUcoMakerDetectionSwapData::new);

      ThreadTools.startAsDaemon(this::detectMarkersOnAsynchronousThread, "ArUcoMarkerDetection");
   }

   private void detectMarkersOnAsynchronousThread()
   {
      while (running)
      {
         OpenCVArUcoMakerDetectionSwapData data = detectionSwapReference.getForThreadTwo();

         throttler.waitAndRun();
         if (timer.isRunning(0.5))
         {
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

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public boolean isEnabled()
   {
      return enabled;
   }
}
