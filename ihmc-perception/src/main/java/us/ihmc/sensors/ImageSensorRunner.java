package us.ihmc.sensors;

import us.ihmc.tools.thread.RestartableThread;

import java.util.HashSet;
import java.util.Set;

public class ImageSensorRunner implements AutoCloseable
{
   private final ImageSensorGrabber imageGrabber;
   private final RestartableThread sensorThread;

   private final Set<Runnable> postGrabRunnables = new HashSet<>();

   public ImageSensorRunner(ImageSensorGrabber imageGrabber)
   {
      this.imageGrabber = imageGrabber;

      sensorThread = new RestartableThread(imageGrabber.getSensorName() + "Runner", this::run);
   }
   
   public void addPostGrabRunnable(Runnable runnable)
   {
      postGrabRunnables.add(runnable);
   }

   public ImageSensorGrabber getImageGrabber()
   {
      return imageGrabber;
   }

   public synchronized boolean start()
   {
      if (isRunning())
         return true;

      if (!imageGrabber.isRunning())
         imageGrabber.startSensor();

      if (imageGrabber.isRunning())
         sensorThread.start();

      return isRunning();
   }

   public synchronized void stop()
   {
      sensorThread.stop();
   }

   @Override
   public void close()
   {
      sensorThread.blockingStop();
   }

   public boolean isRunning()
   {
      return sensorThread.isRunning() && imageGrabber.isRunning();
   }

   private void run()
   {
      // Grab the images
      if (!imageGrabber.grab())
      {
         stop();
         return;
      }

      // Run the algorithms after the grab
      for (Runnable runnable : postGrabRunnables)
         runnable.run();
   }
}
