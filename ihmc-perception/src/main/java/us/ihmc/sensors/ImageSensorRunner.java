package us.ihmc.sensors;

import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.RestartableThread;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.BooleanSupplier;

public class ImageSensorRunner implements AutoCloseable
{
   private static final double SECONDS_BETWEEN_RETRIES = 1.0;  // Wait 1 second between retries for starting sensors

   private final ImageSensorGrabber imageGrabber;
   private final RestartableThread sensorThread;

   private final Executor algorithmExecutor = Executors.newCachedThreadPool();
   private final Map<Runnable, BooleanSupplier> postGrabAlgorithms = new HashMap<>();

   public ImageSensorRunner(ImageSensorGrabber imageGrabber)
   {
      this.imageGrabber = imageGrabber;

      sensorThread = new RestartableThread(imageGrabber.getSensorName() + "Runner", this::run);
   }

   public void addPostGrabAlgorithm(Runnable algorithm)
   {
      addPostGrabAlgorithm(algorithm, () -> true);
   }
   
   public void addPostGrabAlgorithm(Runnable algorithm, BooleanSupplier runAlgorithm)
   {
      postGrabAlgorithms.put(algorithm, runAlgorithm);
   }

   public ImageSensorGrabber getImageGrabber()
   {
      return imageGrabber;
   }

   public synchronized void start()
   {
      sensorThread.start();
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
      // If the sensor is not running, try to start the sensor
      if (!imageGrabber.isRunning() && !imageGrabber.startSensor())
      {  // if sensor failed to start, sleep a bit and retry
         MissingThreadTools.sleep(SECONDS_BETWEEN_RETRIES);
         return;
      }

      // Grab the images
      if (!imageGrabber.grab())
         return; // Grab failed, return

      // Run the algorithms after the grab
      for (Entry<Runnable, BooleanSupplier> algorithmEntry : postGrabAlgorithms.entrySet())
      {
         if (algorithmEntry.getValue().getAsBoolean())
            algorithmExecutor.execute(algorithmEntry.getKey());
      }
   }
}
