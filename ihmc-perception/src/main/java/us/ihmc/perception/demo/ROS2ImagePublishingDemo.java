package us.ihmc.perception.demo;

import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_PERFORMANCE;

import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageSensorPublishThread;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.tools.IHMCCommonPaths;

/**
 * Demo for publishing standard ROS 2 {@link Image} messages along with {@link CameraInfo} messages.
 * This demo does NOT publish IHMC's custom {@link perception_msgs.ImageMessage}.
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
 * rviz2
 * }
 * </pre>
 * In RViz2, add two Image display by clicking the Add button,
 * and in the By topic options selecting the Image displays under
 * the /zed/color/image_raw topic, and again under the /zed/depth/image_raw topic.
 * <p>
 * To show a point cloud, set the Fixed Frame (under Global Options) to "World".
 * Then, click Add, select DepthCloud, and click Ok.
 * Under the DepthCloud options, set the options as such:
 * <ul>
 *    <li>Reliability Policy = Reliable</li>
 *    <li>Depth Map Topic = /zed/depth/image_raw</li>
 *    <li>Color Image Topic = /zed/color/image_raw</li>
 * </ul>
 * <p>
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

   private final ROS2ZEDSVOPlaybackSensor zed;

   // Field for predefined publish thread
   private ImageSensorPublishThread imageSensorPublishThread;

   // Fields for a custom publish thread
   private RawImagePublisher rawImagePublisher;
   private RepeatingTaskThread customPublishThread;

   private ROS2ImagePublishingDemo()
   {
      // Create a ROS 2 Node
      ros2Node = new ROS2Node("image_publishing_demo");

      // ZED topic used to create color and depth topics. QoS must be reliable for RViz2 to receive the images.
      ROS2Topic<?> zedTopic = new ROS2Topic<>().prependedWith("zed");

      // Create the /zed/color/image_raw and /zed/color/camera_info topics
      colorImageTopic = zedTopic.appendedWith("color").appendedWith("image_raw").withType(Image.class);
      colorCameraInfoTopic = zedTopic.appendedWith("color").appendedWith("camera_info").withType(CameraInfo.class);

      // Create the /zed/depth/image_raw and /zed/depth/camera_info topics
      depthImageTopic = zedTopic.appendedWith("depth").appendedWith("image_raw").withType(Image.class);
      depthCameraInfoTopic = zedTopic.appendedWith("depth").appendedWith("camera_info").withType(CameraInfo.class);

      // Create a ZED sensor (in this case we use an SVO playback)
      zed = new ROS2ZEDSVOPlaybackSensor(new ROS2Helper(ros2Node), 0, ZEDModelData.ZED_2, SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
      zed.useTrackedPose(true);

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
      // Create the predefined publish thread
      imageSensorPublishThread = new ImageSensorPublishThread(ros2Node, zed);

      // Give it the topics it should publish on
      imageSensorPublishThread.addTopic(colorImageTopic, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);        // Color image topic requires ZED's left color image
      imageSensorPublishThread.addTopic(colorCameraInfoTopic, ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);   // Color camera info topic requires ZED's left color image
      imageSensorPublishThread.addTopic(depthImageTopic, ZEDImageSensor.DEPTH_IMAGE_KEY);             // Depth image topic requires ZED's depth image
      imageSensorPublishThread.addTopic(depthCameraInfoTopic, ZEDImageSensor.DEPTH_IMAGE_KEY);        // Depth camera info topic requires ZED's depth image

      // Enable ROS 2 frames in the publish thread.
      imageSensorPublishThread.enableROS2Frames(true);

      // We don't need to publish CameraInfo every time, so we set a skip count
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
         rawImagePublisher.publishImage(colorImageTopic, color, ReferenceFrame.getWorldFrame());
         rawImagePublisher.publishImage(depthImageTopic, depth, ReferenceFrame.getWorldFrame());

         // Publish the camera info
         if (color.getSequenceNumber() % (CAMERA_INFO_PUBLISH_SKIP + 1) == 0)
         {
            LogTools.info("Publishing camera info");
            rawImagePublisher.publishImage(colorCameraInfoTopic, color, ReferenceFrame.getWorldFrame());
            rawImagePublisher.publishImage(depthCameraInfoTopic, depth, ReferenceFrame.getWorldFrame());
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
      ros2Node.close();
   }

   public static void main(String[] args)
   {
      new ROS2ImagePublishingDemo();
   }
}
