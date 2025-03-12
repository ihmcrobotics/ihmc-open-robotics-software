package us.ihmc.perception;

import sensor_msgs.msg.dds.CameraInfo;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.packets.Packet;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

public class ImageSensorPublishThread extends RepeatingTaskThread
{
   private final Map<ROS2Topic<? extends Packet<?>>, Integer> topicToImageKeyMap;
   private final Map<ROS2Topic<? extends Packet<?>>, Long> lastPublishedSequenceNumbers = new HashMap<>();
   private final RawImagePublisher publisher;
   private boolean publishingIsThrottled = false;

   private final ImageSensor imageSensor;

   private int cameraInfoPublishModulus = 1;

   public ImageSensorPublishThread(ROS2Node ros2Node, ImageSensor sensorToPublish, Map<ROS2Topic<? extends Packet<?>>, Integer> topicToImageKeyMap)
   {
      super(sensorToPublish.getSensorName() + "PublishThread");

      imageSensor = sensorToPublish;
      this.topicToImageKeyMap = topicToImageKeyMap;
      publisher = new RawImagePublisher(ros2Node);
   }

   @Override
   public ImageSensorPublishThread setFrequencyLimit(double publishFrequencyLimit)
   {
      super.setFrequencyLimit(publishFrequencyLimit);
      publishingIsThrottled = publishFrequencyLimit > 0.0;
      return this;
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

         for (Entry<ROS2Topic<? extends Packet<?>>, Integer> imageEntry : topicToImageKeyMap.entrySet())
         {
            ROS2Topic<? extends Packet<?>> imageTopic = imageEntry.getKey();

            // If the topic is a camera info topic, check whether we should skip this publish
            if (imageTopic.getType().equals(CameraInfo.class) && getCompleted() % cameraInfoPublishModulus != 0)
               continue;

            // Get the image to publish
            int imageKey = imageEntry.getValue();
            RawImage imageToPublish = imageSensor.getImage(imageKey);

            // Skip if the image is null
            if (imageToPublish == null)
               continue;

            // Skip if this image was already published on the topic
            if (lastPublishedSequenceNumbers.containsKey(imageTopic) && imageToPublish.getSequenceNumber() <= lastPublishedSequenceNumbers.get(imageTopic))
               continue;

            // Store the sequence number to avoid re-publishing this image
            lastPublishedSequenceNumbers.put(imageTopic, imageToPublish.getSequenceNumber());

            // Publish the image
            publisher.publishImage(imageTopic, imageToPublish);
            imageToPublish.release();
         }
      }
      catch (InterruptedException ignored) {}
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      publisher.close();
   }
}
