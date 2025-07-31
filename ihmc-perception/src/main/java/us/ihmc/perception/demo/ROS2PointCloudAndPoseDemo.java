package us.ihmc.perception.demo;

import geometry_msgs.msg.dds.PoseStamped;
import sensor_msgs.msg.dds.CameraInfo;
import sensor_msgs.msg.dds.Image;
import sensor_msgs.msg.dds.PointCloud2;
import sensor_msgs.msg.dds.PointField;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageSensorPublishThread;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.RawImagePublisher;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.time.Instant;
import java.util.List;

import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_PERFORMANCE;

/**
 * Demo for publishing standard ROS 2 {@link Image} messages, {@link CameraInfo} messages, {@link PointCloud2} messages, and {@link PoseStamped} messages.
 * Topics published:
 * <ul>
 *    <li>/zed/color/image_raw</li>
 *    <li>/zed/color/camera_info</li>
 *    <li>/zed/depth/image_raw</li>
 *    <li>/zed/depth/camera_info</li>
 *    <li>/zed/pose</li>
 *    <li>/zed/point_cloud</li>
 * </ul>
 *
 * Running this demo requires: CUDA, ZED SDK
 */
class ROS2PointCloudAndPoseDemo
{
   /*
    * Download this file from Google Drive if you do not have it.
    * Place it in ~/.ihmc/logs/perception
    */
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                   .toAbsolutePath()
                                                                                   .toString();

   private static final int CAMERA_INFO_PUBLISH_SKIP = 5; // Publish camera_info messages every 5 frames

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private final ROS2Topic<Image> colorImageTopic;
   private final ROS2Topic<CameraInfo> colorCameraInfoTopic;
   private final ROS2Topic<Image> depthImageTopic;
   private final ROS2Topic<CameraInfo> depthCameraInfoTopic;
   private final ROS2Topic<PoseStamped> poseTopic;
   private final ROS2Topic<PointCloud2> pointCloudTopic;

   private final ZEDSVOPlaybackSensor zed;
   private ImageSensorPublishThread imageSensorPublishThread;
   private final RawImagePublisher rawImagePublisher;
   private final RepeatingTaskThread posePublishThread;
   private final CUDAPointCloudExtractor pointCloudExtractor;
   private final RepeatingTaskThread pointCloudPublishThread;

   private ROS2PointCloudAndPoseDemo()
   {
      // Create a ROS 2 Node
      ros2Node = new ROS2NodeBuilder().build("image_publishing_demo");
      ros2Helper = new ROS2Helper(ros2Node);

      // ZED topic used to create color and depth topics. QoS must be reliable for RViz2 to receive the images.
      ROS2Topic<?> zedTopic = new ROS2Topic<>().withPrefix("zed").withQoS(ROS2QosProfile.RELIABLE());

      // Create the /zed/color/image_raw and /zed/color/camera_info topics
      colorImageTopic = zedTopic.withModule("color").withSuffix("image_raw").withType(Image.class);
      colorCameraInfoTopic = zedTopic.withModule("color").withSuffix("camera_info").withType(CameraInfo.class);

      // Create the /zed/depth/image_raw and /zed/depth/camera_info topics
      depthImageTopic = zedTopic.withModule("depth").withSuffix("image_raw").withType(Image.class);
      depthCameraInfoTopic = zedTopic.withModule("depth").withSuffix("camera_info").withType(CameraInfo.class);

      // Create the /zed/pose topic
      poseTopic = zedTopic.withModule("pose").withType(PoseStamped.class);

      // Create the /zed/point_cloud topic
      pointCloudTopic = zedTopic.withModule("point_cloud").withType(PointCloud2.class);

      // Create a ZED sensor (in this case we use an SVO playback)
      zed = new ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
      zed.useTrackedPose(true);

      // Add shutdown hook to properly close/destroy everything
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Shutdown"));

      // Create a publisher, and the publisher thread
      rawImagePublisher = new RawImagePublisher(ros2Node);

      // Start the sensor and publish threads
      LogTools.info("Starting sensor...");
      zed.run(true);

      startPredefinedPublishThread();

      posePublishThread = new RepeatingTaskThread("PosePublishThread", this::publishPose);
      posePublishThread.startRepeating();

      pointCloudExtractor = new CUDAPointCloudExtractor();

      pointCloudPublishThread = new RepeatingTaskThread("PointCloudPublishThread", this::publishPointCloud);
      pointCloudPublishThread.startRepeating();
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

   private void publishPose()
   {
      try
      {
         // Wait for the ZED to grab images
         zed.waitForGrab();

         PoseStamped poseStamped = new PoseStamped();
         { // Populate poseStamped
            RawImage latestDepthImage = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

            poseStamped.getHeader().setFrameId("World");

            Instant time = latestDepthImage.getAcquisitionTime();
            poseStamped.getHeader().getStamp().setSec((int) time.getEpochSecond());
            poseStamped.getHeader().getStamp().setNanosec(time.getNano());

            poseStamped.getPose().set(zed.getSensorFrame().getTransformToWorldFrame());

            latestDepthImage.release();
         }

         ros2Helper.publish(poseTopic, poseStamped);
      }
      catch (InterruptedException ignored)
      {
      }
   }

   private static final PointField[] pointCloud2Fields = new PointField[3];

   static
   {
      pointCloud2Fields[0] = new PointField();
      pointCloud2Fields[0].setName("x");
      pointCloud2Fields[0].setOffset(0);
      pointCloud2Fields[0].setDatatype(PointField.FLOAT32);
      pointCloud2Fields[0].setCount(1);
      pointCloud2Fields[1] = new PointField();
      pointCloud2Fields[1].setName("y");
      pointCloud2Fields[1].setOffset(4);
      pointCloud2Fields[1].setDatatype(PointField.FLOAT32);
      pointCloud2Fields[1].setCount(1);
      pointCloud2Fields[2] = new PointField();
      pointCloud2Fields[2].setName("z");
      pointCloud2Fields[2].setOffset(8);
      pointCloud2Fields[2].setDatatype(PointField.FLOAT32);
      pointCloud2Fields[2].setCount(1);
   }

   private void publishPointCloud()
   {
      try
      {
         // Wait for the ZED to grab images
         zed.waitForGrab();

         PointCloud2 pointCloudMessage = new PointCloud2();
         { // Populate pointCloudMessage
            RawImage depthImage = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
            List<Point3D32> points = pointCloudExtractor.extractPointCloud(depthImage);

            pointCloudMessage.getHeader().setFrameId("World");
            pointCloudMessage.getHeader().getStamp().setSec((int) depthImage.getAcquisitionTime().getEpochSecond());
            pointCloudMessage.getHeader().getStamp().setNanosec(depthImage.getAcquisitionTime().getNano());

            int pointCount = points.size();
            pointCloudMessage.setHeight(1);
            pointCloudMessage.setWidth(pointCount);
            pointCloudMessage.setIsBigendian(false);
            pointCloudMessage.setIsDense(true);

            pointCloudMessage.getFields().add().set(pointCloud2Fields[0]);
            pointCloudMessage.getFields().add().set(pointCloud2Fields[1]);
            pointCloudMessage.getFields().add().set(pointCloud2Fields[2]);

            int pointStep = 12;  // 3 floats * 4 bytes
            pointCloudMessage.setPointStep(pointStep);
            pointCloudMessage.setRowStep((long) pointStep * pointCount);

            // Pack the points into a byte array
            byte[] data = new byte[pointCount * pointStep];
            ByteBuffer buffer = ByteBuffer.wrap(data);
            buffer.order(ByteOrder.LITTLE_ENDIAN);

            for (Point3D32 point : points)
            {
               buffer.putFloat((float) point.getX());
               buffer.putFloat((float) point.getY());
               buffer.putFloat((float) point.getZ());
            }

            pointCloudMessage.getData().add(data);

            depthImage.release();
         }

         ros2Helper.publish(pointCloudTopic, pointCloudMessage);
      }
      catch (InterruptedException ignored)
      {
      }
   }

   private void destroy()
   {
      posePublishThread.kill();
      posePublishThread.interrupt();
      pointCloudPublishThread.kill();
      pointCloudPublishThread.interrupt();
      rawImagePublisher.close();

      if (imageSensorPublishThread != null)
         imageSensorPublishThread.kill();

      // Stop the ZED
      zed.close();

      // Destroy the ROS2Node
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new ROS2PointCloudAndPoseDemo();
   }
}
