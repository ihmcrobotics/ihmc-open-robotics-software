package us.ihmc.sensors;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.PausableLoopingThread;

import java.util.function.Supplier;

public abstract class ImageSensor implements AutoCloseable
{
   private static final double SECONDS_BETWEEN_RETRIES = 1.0;  // Wait 1 second between retries for starting sensors

   private final String sensorName;
   /** Sensor will be in world frame by default, unless a sensor frame supplier is specified through {@link #setSensorFrameSupplier(Supplier)}. */
   protected volatile Supplier<ReferenceFrame> sensorFrameSupplier;

   private final PausableLoopingThread grabThread;
   private final Object grabNotification = new Object();

   public ImageSensor(String sensorName)
   {
      this.sensorName = sensorName;
      grabThread = new PausableLoopingThread(this::grabAndNotify, sensorName + "Grabber");
   }

   /**
    * Initializes and starts the sensor.
    * @return Whether the sensor was successfully initialized and started.
    */
   protected abstract boolean startSensor();

   /**
    * Set the sensor frame supplier.
    * @param sensorFrameSupplier Supplier of the sensor's reference frame.
    */
   public void setSensorFrameSupplier(Supplier<ReferenceFrame> sensorFrameSupplier)
   {
      this.sensorFrameSupplier = sensorFrameSupplier;
   }

   /**
    * @return Whether the sensor is running. Not necessarily the same as whether the grab thread is running.
    */
   public abstract boolean isSensorRunning();

   /**
    * Grab all images the sensor provides. Blocks until new images are available.
    * @return Whether new images were successfully grabbed.
    */
   protected abstract boolean grab();

   /**
    * <p>
    * Get the latest image grabbed by the sensor, specifying the image to get using its key.
    * </p>
    * <p>
    * This method should either return a new {@link RawImage}, such that the reference count = 1,
    * or increment the reference count of the returned image by calling {@link RawImage#get()}.
    * It is the caller's responsibility to call {@link RawImage#release()} after calling this method.
    * </p>
    * @param imageKey Key of the image to get.
    * @return A {@link RawImage} with an incremented reference count.
    * The caller must call {@link RawImage#release()}.
    */
   public abstract RawImage getImage(int imageKey);

   public String getSensorName()
   {
      return sensorName;
   }

   public synchronized void run(boolean run)
   {
      if (run)
         grabThread.start();
      else
         grabThread.pause();
   }

   public void waitForGrab() throws InterruptedException
   {
      synchronized (grabNotification)
      {
         grabNotification.wait();
      }
   }

   public void waitForGrab(double timeoutSeconds) throws InterruptedException
   {
      long millis = (long) Conversions.secondsToMilliseconds(timeoutSeconds);
      long additionalNanos = Conversions.secondsToNanoseconds(timeoutSeconds) - Conversions.millisecondsToNanoseconds(millis);

      synchronized (grabNotification)
      {
         grabNotification.wait(millis, (int) additionalNanos);
      }
   }

   @Override
   public void close()
   {
      grabThread.blockingDestroy();
   }

   public PausableLoopingThread getGrabThread()
   {
      return grabThread;
   }

   private void grabAndNotify()
   {
      // If the sensor is not running, try to start the sensor
      if (!isSensorRunning() && !startSensor())
      {  // if sensor failed to start, sleep a bit and retry
         MissingThreadTools.sleep(SECONDS_BETWEEN_RETRIES);
         return;
      }

      // Grab the images
      if (!grab())
         return; // Grab failed, return

      // Grab succeeded, notify threads waiting for new images
      synchronized (grabNotification)
      {
         grabNotification.notifyAll();
      }
   }
}
