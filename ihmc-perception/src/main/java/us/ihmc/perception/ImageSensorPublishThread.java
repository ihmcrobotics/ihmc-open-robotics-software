package us.ihmc.perception;

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
   private final Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap;
   private final Map<Integer, Long> lastPublishedSequenceNumbers = new HashMap<>();
   private final RawImagePublisher publisher;
   private boolean publishingIsThrottled = false;

   private final ImageSensor imageSensor;

   public ImageSensorPublishThread(ROS2Node ros2Node, ImageSensor sensorToPublish, Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap)
   {
      super(sensorToPublish.getSensorName() + "PublishThread");

      imageSensor = sensorToPublish;
      this.imageKeyToTopicMap = imageKeyToTopicMap;
      publisher = new RawImagePublisher(ros2Node);
   }

   public ImageSensorPublishThread(ROS2Node ros2Node,
                                   ImageSensor sensorToPublish,
                                   Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap,
                                   double publishFrequencyLimit)
   {
      this(ros2Node, sensorToPublish, imageKeyToTopicMap);
      setFrequencyLimit(publishFrequencyLimit);
   }

   @Override
   public ImageSensorPublishThread setFrequencyLimit(double publishFrequencyLimit)
   {
      super.setFrequencyLimit(publishFrequencyLimit);
      publishingIsThrottled = publishFrequencyLimit > 0.0;
      return this;
   }

   @Override
   protected void runTask()
   {
      try
      {  // If publisher frequency is not set, run at sensor's frequency
         if (!publishingIsThrottled)
            imageSensor.waitForGrab();

         for (Entry<Integer, ROS2Topic<? extends Packet<?>>> imageEntry : imageKeyToTopicMap.entrySet())
         {
            int imageKey = imageEntry.getKey();
            ROS2Topic<? extends Packet<?>> imageTopic = imageEntry.getValue();

            // Get the image to publish
            RawImage imageToPublish = imageSensor.getImage(imageKey);

            // Skip if the image is null
            if (imageToPublish == null)
               continue;

            // Skip if this image was already published
            if (lastPublishedSequenceNumbers.containsKey(imageKey) && imageToPublish.getSequenceNumber() == lastPublishedSequenceNumbers.get(imageKey))
               continue;

            // Store the sequence number to avoid re-publishing this image
            lastPublishedSequenceNumbers.put(imageKey, imageToPublish.getSequenceNumber());

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
