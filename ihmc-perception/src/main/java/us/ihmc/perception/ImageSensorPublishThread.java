package us.ihmc.perception;

import us.ihmc.communication.packets.Packet;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.thread.RestartableThread;

import java.util.Map;
import java.util.Map.Entry;

public class ImageSensorPublishThread // extends PausableLoopingThread
{
   private final Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap;

   private final ImageSensor imageSensor;
   private final RawImagePublisher publisher;

   private volatile boolean stop = false;

   private final RestartableThread publishThread;

   public ImageSensorPublishThread(ROS2Node ros2Node, ImageSensor sensorToPublish, Map<Integer, ROS2Topic<? extends Packet<?>>> imageKeyToTopicMap)
   {
      imageSensor = sensorToPublish;
      this.imageKeyToTopicMap = imageKeyToTopicMap;
      publisher = new RawImagePublisher(ros2Node);

      publishThread = new RestartableThread(imageSensor.getSensorName() + "PublishThread", this::runInLoop);
   }

   // @Override
   public void runInLoop()
   {
      if (stop)
         return;

      try
      {  // Wait for images to be grabbed
         imageSensor.waitForGrab();

         for (Entry<Integer, ROS2Topic<? extends Packet<?>>> imageEntry : imageKeyToTopicMap.entrySet())
         {
            int imageKey = imageEntry.getKey();
            ROS2Topic<? extends Packet<?>> imageTopic = imageEntry.getValue();

            RawImage imageToPublish = imageSensor.getImage(imageKey);
            publisher.publishImage(imageTopic, imageToPublish, imageSensor.getCameraModel());
            imageToPublish.release();
         }
      }
      catch (InterruptedException stopSignal)
      {
         stop = true;
      }
   }

   // @Override
   public void close()
   {
      interrupt();
      // super.close();
      publishThread.stop();

      publisher.close();
   }

   // TODO: get rid of duplicate code
   public void start()
   {
      publishThread.start();
   }

   public void pause()
   {
      publishThread.stop();
   }

   public void interrupt()
   {
      publishThread.interrupt();
   }
}
