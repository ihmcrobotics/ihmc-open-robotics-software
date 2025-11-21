package us.ihmc.sensors;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;

public abstract class ImageSensor implements AutoCloseable
{
   public static final int DEFAULT_IMAGE_QUEUE_CAPACITY = 8;
   private static final double SECONDS_BETWEEN_RETRIES = 1.0;  // Wait 1 second between retries for starting sensors

   private final String sensorName;
   /** Sensor will be in world frame by default, unless a sensor frame is specified through {@link #setSensorFrame(ReferenceFrame)}. */
   protected ReferenceFrame sensorFrame = ReferenceFrame.getWorldFrame();

   private final RepeatingTaskThread grabThread;
   private final Object grabNotification = new Object();
   private final Map<Integer, List<BlockingQueue<RawImage>>> imageQueues = new HashMap<>();

   public ImageSensor(String sensorName)
   {
      this.sensorName = sensorName;
      grabThread = new RepeatingTaskThread(sensorName + "Grabber", this::grabAndNotify);
   }

   /**
    * Set the sensor frame.
    * @param sensorFrame The sensor's reference frame.
    */
   public void setSensorFrame(ReferenceFrame sensorFrame)
   {
      if (this.sensorFrame != sensorFrame)
      {
         this.sensorFrame = sensorFrame;
         onSensorFrameChanged();
      }
   }

   protected void onSensorFrameChanged()
   {
      // To be optionally implemented by subclasses
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }

   /**
    * Initializes and starts the sensor.
    * @return Whether the sensor was successfully initialized and started.
    */
   protected abstract boolean startSensor();

   /**
    * @return Whether the sensor is running. Not necessarily the same as whether the grab thread is running.
    */
   public abstract boolean isSensorRunning();

   /**
    * Grab all images the sensor provides. Blocks until new images are available.
    * @return Whether new images were successfully grabbed.
    */
   protected abstract boolean grab();

   public abstract int[] getImageKeys();

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

   /**
    * Get the {@link ReferenceFrame} associated with an image, specified by the image key.
    *
    * @param imageKey Key of the image associated with the reference frame.
    * @return The {@link ReferenceFrame} of the image. {@code null} if an invalid key is provided.
    */
   public abstract ReferenceFrame getImageFrame(int imageKey);

   /**
    * Get all image reference frames provided by the sensor.
    *
    * @return An array of all image reference frames provided by the sensor.
    * @apiNote The number of reference might not equal the number of images as some images may share a frame.
    *       Image keys do not work on the returned array.
    */
   public abstract ReferenceFrame[] getImageFrames();

   /**
    * Register an image queue for images of a particular key.
    * <p>
    * Every image grabbed by the sensor will be added to the passed in queue.
    * The code taking from the queue must call {@link RawImage#release()} on each image,
    * once it's done using the image.
    * <p>
    * If the queue becomes full (i.e. when {@code BlockingQueue#remainingCapacity() == 0})
    * the oldest image will be removed so the new image can be added.
    * Ensure a reasonable queue capacity is set to prevent memory leaks.
    * {@link #DEFAULT_IMAGE_QUEUE_CAPACITY} can be used as a good default value.
    *
    * @param imageQueue Blocking queue into which the images will be added.
    * @param imageKey The key for images to be collected.
    */
   public void registerImageQueue(BlockingQueue<RawImage> imageQueue, int imageKey)
   {
      if (imageQueues.containsKey(imageKey))
         imageQueues.get(imageKey).add(imageQueue);
      else
         imageQueues.put(imageKey, new ArrayList<>(List.of(imageQueue)));
   }

   public String getSensorName()
   {
      return sensorName;
   }

   public synchronized void run(boolean run)
   {
      if (!grabThread.isAlive())
         grabThread.start();

      grabThread.setRepeating(run);
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
      grabThread.blockingKill();
   }

   public RepeatingTaskThread getGrabThread()
   {
      return grabThread;
   }

   /** Call to grab a frame without using the thread. */
   public void grabAndNotify()
   {
      // If the sensor is not running, try to start the sensor
      if (!isSensorRunning() && !startSensor())
      {  // if sensor failed to start, sleep a bit and retry
         ThreadTools.park(SECONDS_BETWEEN_RETRIES);
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

      for (int imageKey : getImageKeys())
      {
         if (imageQueues.containsKey(imageKey))
         {
            imageQueues.get(imageKey).forEach(queue ->
            {
               RawImage image = getImage(imageKey);

               if (image == null)
                   return;

               if (!queue.offer(image))
               {
                  // Meaning we couldn't add the image to the queue
                  // We need to remove an item from the queue to create space
                  RawImage oldestImage = queue.poll();
                  if (oldestImage != null)
                     oldestImage.release();

                  // Try to add image one more...
                  if (!queue.offer(image))
                  {
                     // Still couldn't add an image to the queue, release image and forget about it
                     image.release();
                  }
               }
            });
         }
      }
   }
}
