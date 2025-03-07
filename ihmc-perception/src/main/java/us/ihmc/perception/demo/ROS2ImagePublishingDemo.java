package us.ihmc.perception.demo;

import sensor_msgs.msg.dds.Image;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_PERFORMANCE;

class ROS2ImagePublishingDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                   .toAbsolutePath()
                                                                                   .toString();

   private final ROS2Node ros2Node;
   private final ROS2Topic<Image> colorTopic;
   private final ROS2Topic<Image> depthTopic;

   private final RawImagePublisher rawImagePublisher;
   private final RepeatingTaskThread publishThread;

   private final ZEDSVOPlaybackSensor zed;

   private ROS2ImagePublishingDemo()
   {
      ros2Node = new ROS2NodeBuilder().build("image_publishing_demo");

      colorTopic = new ROS2Topic<>().withPrefix("zed").withModule("color").withQoS(ROS2QosProfile.RELIABLE()).withType(Image.class);
      depthTopic = new ROS2Topic<>().withPrefix("zed").withModule("depth").withQoS(ROS2QosProfile.RELIABLE()).withType(Image.class);

      rawImagePublisher = new RawImagePublisher(ros2Node);
      publishThread = new RepeatingTaskThread("SVOPublishThread", this::publishSensorImages);

      zed = new ZEDSVOPlaybackSensor(new ROS2Helper(ros2Node), 0, ZEDModelData.ZED_2, SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));

      LogTools.info("Starting sensor...");
      publishThread.startRepeating();
      zed.run(true);
   }

   private void publishSensorImages()
   {
      try
      {
         zed.waitForGrab();

         RawImage color = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage depth = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

         LogTools.info("Publishing images {}", color.getSequenceNumber());
         rawImagePublisher.publishImage(colorTopic, color);
         rawImagePublisher.publishImage(depthTopic, depth);

         color.release();
         depth.release();
      }
      catch (InterruptedException ignored) {}
   }

   private void destroy()
   {
      publishThread.kill();
      publishThread.interrupt();

      zed.close();

      ros2Node.destroy();
      rawImagePublisher.close();
   }

   public static void main(String[] args)
   {
      new ROS2ImagePublishingDemo();
   }
}
