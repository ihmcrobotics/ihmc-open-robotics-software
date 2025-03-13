package us.ihmc.perception.demo;

import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageSensorPublishThread;
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

import java.util.Map;

import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_PERFORMANCE;

/**
 * Demo for publishing standard ROS 2 {@link Image} messages along with {@link CameraInfo} messages.
 * This demo does NOT publish IHMC's custom {@link perception_msgs.msg.dds.ImageMessage}.
 * <p>
 * The demo publishes messages on the following topics:
 * <ul>
 *    <li>/zed/color/image_raw</li>
 *    <li>/zed/color/camera_info</li>
 *    <li>/zed/depth/image_raw</li>
 *    <li>/zed/depth/camera_info</li>
 * </ul>
 * <p>
 * To visualize these images, you may use RViz2 from ROS 2
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
 * the /zed/color/image_raw topic, and again under the /zed/depth/image_raw topic.
 * <p>
 * To show a point cloud, set the Fixed Frame (under Global Options) to "world".
 * Then, click Add, select DepthCloud, and click Ok.
 * Under the DepthCloud options, set the options as such:
 * <ul>
 *    <li>Reliability Policy = Reliable</li>
 *    <li>Depth Map Topic = /zed/depth/image_raw</li>
 *    <li>Color Image Topic = /zed/color/image_raw</li>
 * </ul>
 * <p>
 * You may notice the orientation of the point cloud is incorrect
 * as this demo does not publish a tf tree.
 */
class ROS2ImagePublishingDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                   .toAbsolutePath()
                                                                                   .toString();

   private static final int CAMERA_INFO_PUBLISH_SKIP = 5; // Publish camera_info messages every 5 frames

   private final ROS2Node ros2Node;

   private final ROS2Topic<Image> colorImageTopic;
   private final ROS2Topic<CameraInfo> colorCameraInfoTopic;

   private final ROS2Topic<Image> depthImageTopic;
   private final ROS2Topic<CameraInfo> depthCameraInfoTopic;

   private final ZEDSVOPlaybackSensor zed;

   // Field for predefined publish thread
   private ImageSensorPublishThread imageSensorPublishThread;

   // Fields for a custom publish thread
   private RawImagePublisher rawImagePublisher;
   private RepeatingTaskThread customPublishThread;

   private ROS2ImagePublishingDemo()
   {
      // Create a ROS 2 Node
      ros2Node = new ROS2NodeBuilder().build("image_publishing_demo");

      // ZED topic used to create color and depth topics. QoS must be reliable for RViz2 to receive the images.
      ROS2Topic<?> zedTopic = new ROS2Topic<>().withPrefix("zed").withQoS(ROS2QosProfile.RELIABLE());

      // Create the /zed/color/image_raw and /zed/color/camera_info topics
      colorImageTopic = zedTopic.withModule("color").withSuffix("image_raw").withType(Image.class);
      colorCameraInfoTopic = zedTopic.withModule("color").withSuffix("camera_info").withType(CameraInfo.class);

      // Create the /zed/depth/image_raw and /zed/depth/camera_info topics
      depthImageTopic = zedTopic.withModule("depth").withSuffix("image_raw").withType(Image.class);
      depthCameraInfoTopic = zedTopic.withModule("depth").withSuffix("camera_info").withType(CameraInfo.class);

      // Create a ZED sensor (in this case we use an SVO playback)
      zed = new ZEDSVOPlaybackSensor(new ROS2Helper(ros2Node), 0, ZEDModelData.ZED_2, SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);

      // Add shutdown hook to properly close/destroy everything
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));

      // Create a publisher, and the publisher thread
      rawImagePublisher = new RawImagePublisher(ros2Node);
      customPublishThread = new RepeatingTaskThread("SVOPublishThread", this::publishSensorImages);

      // Start the sensor and publish threads
      LogTools.info("Starting sensor...");
      zed.run(true);

      // Uncomment the method for the thread you want to run
      startPredefinedPublishThread();
//      startCustomPublishThread();
   }

   private void startPredefinedPublishThread()
   {
      // Define a topic to image key map
      Map<ROS2Topic<? extends Packet<?>>, Integer> zedTopicMap =
            Map.of(colorImageTopic, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY,       // Color image topic requires ZED's left color image
                   colorCameraInfoTopic, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY,  // Color camera info topic requires ZED's left color image
                   depthImageTopic, ZEDImageSensor.DEPTH_IMAGE_KEY,            // Depth image topic requires ZED's depth image
                   depthCameraInfoTopic, ZEDImageSensor.DEPTH_IMAGE_KEY);      // Depth camera info topic requires ZED's depth image

      // Create the predefined publish thread and set its parameters
      imageSensorPublishThread = new ImageSensorPublishThread(ros2Node, zed, zedTopicMap);
      imageSensorPublishThread.setCameraInfoPublishGrabSkipCount(CAMERA_INFO_PUBLISH_SKIP);

      // Start the thread
      imageSensorPublishThread.startRepeating();
   }

   private void startCustomPublishThread()
   {
      // Create a publisher, and the publisher thread
      rawImagePublisher = new RawImagePublisher(ros2Node);
      customPublishThread = new RepeatingTaskThread("SVOPublishThread", this::publishSensorImages);

      // start repeating
      customPublishThread.startRepeating();
   }

   /**
    * The custom publish method (a simplified version of what the predefined one does).
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
         rawImagePublisher.publishImage(colorImageTopic, color);
         rawImagePublisher.publishImage(depthImageTopic, depth);

         // Publish the camera info
         if (color.getSequenceNumber() % (CAMERA_INFO_PUBLISH_SKIP + 1) == 0)
         {
            LogTools.info("Publishing camera info");
            rawImagePublisher.publishImage(colorCameraInfoTopic, color);
            rawImagePublisher.publishImage(depthCameraInfoTopic, depth);
         }

         // Release the images
         color.release();
         depth.release();
      }
      catch (InterruptedException ignored) {}
   }

   private void destroy()
   {
      // If we used the custom publish thread, kill it
      if (customPublishThread != null)
      {
         customPublishThread.kill();
         customPublishThread.interrupt();
         rawImagePublisher.close();
      }

      // If we used the predefined publish thread, kill it
      if (imageSensorPublishThread != null)
         imageSensorPublishThread.kill();

      // Stop the ZED
      zed.close();

      // Destroy the ROS2Node
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new ROS2ImagePublishingDemo();
   }
}
