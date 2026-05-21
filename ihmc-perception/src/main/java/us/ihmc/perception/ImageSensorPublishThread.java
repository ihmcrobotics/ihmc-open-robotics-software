package us.ihmc.perception;

import org.apache.logging.log4j.core.util.ExecutorServices;
import sensor_msgs.CameraInfo;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ros2.tf2.ROS2FollowingFrame;
import us.ihmc.communication.ros2.tf2.ROS2StaticFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.CAMERA_TO_OPTICAL_TRANSFORM;

public class ImageSensorPublishThread extends RepeatingTaskThread
{
   private final ROS2Node ros2Node;

   private final Map<AsyncImagePublisher, Integer> publisherMap = new HashMap<>();
   private boolean publishingIsThrottled = false;

   private final ImageSensor imageSensor;

   private final Map<Integer, ROS2FollowingFrame> ros2CameraFrames = new HashMap<>();
   private final Map<Integer, ROS2StaticFrame> ros2OpticalFrames = new HashMap<>();

   private boolean ros2FramesEnabled = false;
   private int cameraInfoPublishModulus = 1;

   public ImageSensorPublishThread(ROS2Node ros2Node, ImageSensor sensorToPublish)
   {
      super(sensorToPublish.getSensorName() + "PublishThread");

      this.ros2Node = ros2Node;

      imageSensor = sensorToPublish;
   }

   public void addTopic(ROS2Topic<? extends ROS2Message<?>> topicToPublishOn, int imageKey)
   {
      addTopic(topicToPublishOn, imageKey, 1.0);
   }

   public void addTopic(ROS2Topic<? extends ROS2Message<?>> topicToPublishOn, int imageKey, double scale)
   {
      publisherMap.put(new AsyncImagePublisher(ros2Node, topicToPublishOn, scale), imageKey);
   }

   @Override
   public ImageSensorPublishThread setFrequencyLimit(double publishFrequencyLimit)
   {
      super.setFrequencyLimit(publishFrequencyLimit);
      publishingIsThrottled = publishFrequencyLimit > 0.0;
      return this;
   }

   /**
    * If enabled, this thread will manage a ROS 2 frame for each image frame,
    * and a corresponding optical frame.
    * <p>
    * For more info on optical frames, see
    * <a href="https://ros.org/reps/rep-0103.html">ROS REP 103 - Standard Units of Measure and Coordinate Conventions</a>.
    *
    * @param enable Whether to enable adding ROS 2 frames for the image frames.
    */
   public void enableROS2Frames(boolean enable)
   {
      ros2FramesEnabled = enable;
   }

   /**
    * Set the number of grabs to skip between publishing {@link CameraInfo} messages.
    * If set to zero, a message will be published every grab (default).
    * If set to non-zero, {@code skip} number of grabs will be skipped after publishing a message.
    * Do not set to a negative value.
    *
    * @param skip Number of grabs to skip between publishing {@link CameraInfo} messages.
    */
   public void setCameraInfoPublishGrabSkipCount(int skip)
   {
      cameraInfoPublishModulus = skip + 1;
   }

   @Override
   protected void runTask()
   {
      try
      {  // If publisher frequency is not set, run at sensor's frequency
         if (!publishingIsThrottled)
            imageSensor.waitForGrab();

         for (Entry<AsyncImagePublisher, Integer> imageEntry : publisherMap.entrySet())
         {
            AsyncImagePublisher publisher = imageEntry.getKey();
            ROS2Topic<? extends ROS2Message<?>> imageTopic = publisher.topic;

            // If the topic is a camera info topic, check whether we should skip this publish
            if (imageTopic.getType().equals(CameraInfo.class) && getCompleted() % cameraInfoPublishModulus != 0)
               continue;

            // Get the image to publish
            int imageKey = imageEntry.getValue();
            RawImage imageToPublish = imageSensor.getImage(imageKey);

            // Skip if the image is null
            if (imageToPublish == null)
               continue;

            // Update sensor frames
            ReferenceFrame imageFrame = imageSensor.getImageFrame(imageKey);
            if (ros2FramesEnabled)
            {
               updateROS2Frames(imageKey);
               imageFrame = ros2OpticalFrames.get(imageKey);
            }

            // Publish the image
            publisher.publish(imageToPublish, imageFrame);
            imageToPublish.release();
         }
      }
      catch (InterruptedException ignored) {}
   }

   private void updateROS2Frames(int imageKey)
   {
      ReferenceFrame imageFrame = imageSensor.getImageFrame(imageKey);

      ROS2FollowingFrame cameraFrame = ros2CameraFrames.get(imageKey);
      ROS2StaticFrame opticalFrame = ros2OpticalFrames.get(imageKey);

      // Ensure we've added these frames
      if (cameraFrame == null)
      {
         cameraFrame = new ROS2FollowingFrame("ros2_" + imageFrame.getName(), imageSensor.getSensorFrame(), imageFrame);
         ros2CameraFrames.put(imageKey, cameraFrame);
      }

      if (opticalFrame == null)
      {
         opticalFrame = new ROS2StaticFrame(cameraFrame.getFrameId() + "_optical", cameraFrame, CAMERA_TO_OPTICAL_TRANSFORM);
         ros2OpticalFrames.put(imageKey, opticalFrame);
      }

      // If sensor frame changed, update our frames as well
      if (!cameraFrame.getParent().equals(imageSensor.getSensorFrame()))
      {
         cameraFrame.remove();
         opticalFrame.remove();

         cameraFrame = new ROS2FollowingFrame("ros2_" + imageFrame.getName(), imageSensor.getSensorFrame(), imageFrame);
         opticalFrame = new ROS2StaticFrame(cameraFrame.getFrameId() + "_optical", cameraFrame, CAMERA_TO_OPTICAL_TRANSFORM);

         ros2CameraFrames.replace(imageKey, cameraFrame);
         ros2OpticalFrames.replace(imageKey, opticalFrame);
      }

      try
      {
         cameraFrame.update();
         opticalFrame.update();
      }
      catch (Exception exception)
      {
         LogTools.error(exception);
      }
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      publisherMap.keySet().forEach(AsyncImagePublisher::close);
      ros2CameraFrames.values().forEach(ReferenceFrame::remove);
      ros2OpticalFrames.values().forEach(ReferenceFrame::remove);
   }

   private static class AsyncImagePublisher
   {
      private final RawImagePublisher publisher;
      private final ROS2Topic<? extends ROS2Message<?>> topic;
      private long lastSequenceNumber = -1L;

      private final ExecutorService publishExecutor;
      private Future<?> publishFuture;

      private AsyncImagePublisher(ROS2Node ros2Node, ROS2Topic<? extends ROS2Message<?>> topic, double scale)
      {
         this.topic = topic;
         publisher = new RawImagePublisher(ros2Node, scale);
         publishExecutor = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(topic.getName() + getClass().getSimpleName()));
      }

      private void publish(RawImage image, ReferenceFrame sensorFrame)
      {
         // Don't publish if an image is currently being published, or an image with a greater sequence number has already been published
         if ((publishFuture != null && !publishFuture.isDone()) || image.getSequenceNumber() < lastSequenceNumber)
            return;

         image.get();
         lastSequenceNumber = image.getSequenceNumber();
         publishFuture = publishExecutor.submit(() ->
         {
            try
            {
               publisher.publishImage(topic, image, sensorFrame);
            }
            catch (Exception e)
            {
               LogTools.error(e);
            }

            image.release();
         });
      }

      public void close()
      {
         ExecutorServices.shutdown(publishExecutor, 1, TimeUnit.SECONDS, getClass().getSimpleName());
         publisher.close();
      }
   }
}
