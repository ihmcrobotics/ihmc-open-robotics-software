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

/**
 * Demo for publishing standard ROS 2 {@link Image} messages.
 * This demo does NOT publish IHMC's custom {@link perception_msgs.msg.dds.ImageMessage}.
 * <p>
 * The demo publishes color and depth images on the following topics:
 * <ul>
 *    <li>/zed/color</li>
 *    <li>/zed/depth</li>
 * </ul>
 * <p>
 * To visualize these images, you may use RViz2
 * (<a href="https://docs.ros.org/en/humble/Installation.html">ROS 2 Humble Installation</a>).
 * <p>
 * With this demo running, launch RViz2. On Linux, open a new terminal and run:
 * <pre>
 * {@code
 * source /opt/ros/humble/setup.bash
 * export ROS_DOMAIN_ID=[your-domain-id] # insert your domain ID
 * ros2 run rviz2 rviz2
 * }
 * </pre>
 * In RViz2, add two Image display by clicking the Add button,
 * and in the By topic options selecting the Image displays under
 * the /zed/color topic, and again under the /zed/depth topic.
 */
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
      // Create a ROS 2 Node
      ros2Node = new ROS2NodeBuilder().build("image_publishing_demo");

      // Create the /zed/color topic. QoS must be reliable for RViz2 to receive the images.
      colorTopic = new ROS2Topic<>().withModule("zed").withSuffix("color").withQoS(ROS2QosProfile.RELIABLE()).withType(Image.class);

      // Create the /zed/depth topic. Again, QoS must be reliable.
      depthTopic = new ROS2Topic<>().withModule("zed").withSuffix("depth").withQoS(ROS2QosProfile.RELIABLE()).withType(Image.class);

      // Create a publisher, and the publisher thread
      rawImagePublisher = new RawImagePublisher(ros2Node);
      publishThread = new RepeatingTaskThread("SVOPublishThread", this::publishSensorImages);

      // Create a ZED sensor (in this case we use an SVO playback)
      zed = new ZEDSVOPlaybackSensor(new ROS2Helper(ros2Node), 0, ZEDModelData.ZED_2, SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);

      // Add shutdown hook to properly close/destroy everything
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));

      // Start the sensor and publish threads
      LogTools.info("Starting sensor...");
      publishThread.startRepeating();
      zed.run(true);
   }

   /**
    * This method runs in a loop to publish the images acquired from the ZED
    */
   private void publishSensorImages()
   {
      try
      {
         // Wait for the ZED to grab images
         zed.waitForGrab();

         // Get the images
         RawImage color = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage depth = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

         // Publish the images
         LogTools.info("Publishing images {}", color.getSequenceNumber());
         rawImagePublisher.publishImage(colorTopic, color);
         rawImagePublisher.publishImage(depthTopic, depth);

         // Release the images
         color.release();
         depth.release();
      }
      catch (InterruptedException ignored) {}
   }

   private void destroy()
   {
      // Stop the publisher thread
      publishThread.kill();
      publishThread.interrupt();

      // Stop the ZED
      zed.close();

      // Destroy the ROS2Node and publisher
      ros2Node.destroy();
      rawImagePublisher.close();
   }

   public static void main(String[] args)
   {
      new ROS2ImagePublishingDemo();
   }
}
