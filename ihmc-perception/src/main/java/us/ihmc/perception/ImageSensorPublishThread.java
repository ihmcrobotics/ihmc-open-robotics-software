package us.ihmc.perception;

import us.ihmc.communication.packets.Packet;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.thread.PausableLoopingThread;

import java.util.Map;
import java.util.Map.Entry;

public class ImageSensorPublishThread extends PausableLoopingThread
{
   private final Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap;
   private final RawImagePublisher publisher;
   private final ImageSensor imageSensor;

   public ImageSensorPublishThread(ROS2Node ros2Node, ImageSensor sensorToPublish, Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap)
   {
      super(sensorToPublish.getSensorName() + "PublishThread");

      imageSensor = sensorToPublish;
      this.imageKeyToTopicMap = imageKeyToTopicMap;
      publisher = new RawImagePublisher(ros2Node);
   }

   @Override
   public void runInLoop()
   {
      if (isDestroyed())
         return;

      try
      {  // Wait for images to be grabbed
         imageSensor.waitForGrab();

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
   public void destroy()
   {
      super.destroy();
      interrupt();

      publisher.close();
   }
}
