package us.ihmc.perception;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.packets.Packet;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;

import java.util.Map;
import java.util.Map.Entry;

public class ImageSensorPublishThread extends RepeatingTaskThread
{
   private final Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap;
   private final RawImagePublisher publisher;
   private final Throttler publishThrottler;
   private double publishPeriodLimit = -1.0;

   private final ImageSensor imageSensor;

   public ImageSensorPublishThread(ROS2Node ros2Node, ImageSensor sensorToPublish, Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap)
   {
      super(sensorToPublish.getSensorName() + "PublishThread");

      imageSensor = sensorToPublish;
      this.imageKeyToTopicMap = imageKeyToTopicMap;
      publisher = new RawImagePublisher(ros2Node);
      publishThrottler = new Throttler();
   }

   @Override
   public ImageSensorPublishThread setFrequencyLimit(double publishFrequencyLimit)
   {
      publishPeriodLimit = Conversions.hertzToSeconds(publishFrequencyLimit);
      return this;
   }

   @Override
   protected void runTask()
   {
      try
      {  // Wait for images to be grabbed
         imageSensor.waitForGrab();

         // Check if we should publish
         if (publishPeriodLimit > 0.0 && !publishThrottler.run(publishPeriodLimit))
            return;

         for (Entry<Integer, ROS2Topic<? extends Packet<?>>> imageEntry : imageKeyToTopicMap.entrySet())
         {
            int imageKey = imageEntry.getKey();
            ROS2Topic<? extends Packet<?>> imageTopic = imageEntry.getValue();

            RawImage imageToPublish = imageSensor.getImage(imageKey);
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
