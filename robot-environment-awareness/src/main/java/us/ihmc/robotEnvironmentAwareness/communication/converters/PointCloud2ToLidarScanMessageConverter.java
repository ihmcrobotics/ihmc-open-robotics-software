package us.ihmc.robotEnvironmentAwareness.communication.converters;

import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import sensor_msgs.PointCloud2;
import org.jboss.netty.buffer.ChannelBuffer;

import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class will listens for PointCloud2 messages on ROS2 and convert/stream them in real time as LidarScanMessages
 *
 * The PointCloud2 topic and sensor pose can be hard-coded here or passed in as arguments.
 * For the latter, the program arguments are [topicName] [poseX] [poseY] [poseZ] [poseYaw] [posePitch] [poseRoll]
 * For example, program arguments: /os1_cloud_node/points 0.0 0.0 0.0 0.0 0.785 0.0
 */
public class PointCloud2ToLidarScanMessageConverter
{
//   private static final String POINT_CLOUD_2_TOPIC_NAME = "/os_cloud_node/points";
   private static final String POINT_CLOUD_2_TOPIC_NAME = "/camera/depth/color/points";

   private static final double SENSOR_POSE_X = 0.0;
   private static final double SENSOR_POSE_Y = 0.0;
   private static final double SENSOR_POSE_Z = 1.0;
   private static final double SENSOR_POSE_YAW = 0.0;
   private static final double SENSOR_POSE_PITCH = 0.0;
   private static final double SENSOR_POSE_ROLL = -2.35;

   private final AtomicReference<PointCloud2> pointCloud2Message = new AtomicReference<>();
   private final AtomicInteger counter = new AtomicInteger();
   private final AtomicBoolean firstMessage = new AtomicBoolean(true);

   public PointCloud2ToLidarScanMessageConverter(String topicName, double poseX, double poseY, double poseZ, double poseYaw, double posePitch, double poseRoll)
         throws URISyntaxException
   {
      URI rosMasterURI = new URI("http://localhost:11311");
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, getClass().getSimpleName());
      Pose3D sensorPose = new Pose3D(poseX, poseY, poseZ, poseYaw, posePitch, poseRoll);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
      ROS2PublisherBasics<LidarScanMessage> lidarScanMessagePublisher = ROS2Tools.createPublisher(ros2Node, LidarScanMessage.class, "/ihmc/lidar_scan");

      // save and handle on another thread
      RosPointCloudSubscriber pointCloudSubscriber = new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(sensor_msgs.PointCloud2 pointCloud)
         {
            pointCloud2Message.set(pointCloud);
         }
      };

      LidarScanMessage lidarScanMessage = new LidarScanMessage();
      lidarScanMessage.getLidarPosition().set(sensorPose.getPosition());
      lidarScanMessage.getLidarOrientation().set(sensorPose.getOrientation());

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(poseX, poseY, poseZ);
      transform.getRotation().setYawPitchRoll(poseYaw, posePitch, poseRoll);

      new Thread(() ->
                 {
                    while (true)
                    {
                       PointCloud2 pointCloud = this.pointCloud2Message.getAndSet(null);

                       if (pointCloud != null)
                       {
                          if (firstMessage.getAndSet(false))
                             LogTools.info("Receiving point cloud message...");

                          ByteOrder byteOrder = pointCloud.getIsBigendian() ? ByteOrder.BIG_ENDIAN : ByteOrder.LITTLE_ENDIAN;
                          long pointStep = pointCloud.getPointStep();
                          int width = pointCloud.getWidth();
                          int height = pointCloud.getHeight();
                          int numberOfPoints = width * height;

                          float[] points = new float[3 * numberOfPoints];
                          byte[] bytes = new byte[4];

                          // assumes each point is 4 bytes and the first three fields at the x,y,z positions
                          for (int i = 0; i < numberOfPoints; i++)
                          {
                             int startIndex = (int) pointStep * i;
                             packBytes(bytes, startIndex + 0, pointCloud.getData());
                             float x = ByteBuffer.wrap(bytes).order(byteOrder).getFloat();
                             packBytes(bytes, startIndex + 4, pointCloud.getData());
                             float y = ByteBuffer.wrap(bytes).order(byteOrder).getFloat();
                             packBytes(bytes, startIndex + 8, pointCloud.getData());
                             float z = ByteBuffer.wrap(bytes).order(byteOrder).getFloat();

                             Pose3D dataPoint = new Pose3D(x, y, z, 0.0, 0.0, 0.0);
                             dataPoint.applyTransform(transform);

                             points[3 * i + 0] = (float) dataPoint.getX();
                             points[3 * i + 1] = (float) dataPoint.getY();
                             points[3 * i + 2] = (float) dataPoint.getZ();
                          }

                          lidarScanMessage.setSequenceId(counter.getAndIncrement());
                          lidarScanMessage.getScan().clear();
                          LidarPointCloudCompression.compressPointCloud(numberOfPoints, lidarScanMessage, (index, element) -> points[3 * index + element]);
                          lidarScanMessagePublisher.publish(lidarScanMessage);
                       }

                       ThreadTools.sleep(20);
                    }
                 }).start();

      rosMainNode.attachSubscriber(topicName, pointCloudSubscriber);
      rosMainNode.execute();
      pointCloudSubscriber.wailTillRegistered();
   }

   private static void packBytes(byte[] bytes, int startIndex, ChannelBuffer data)
   {
      for (int i = 0; i < 4; i++)
      {
         bytes[i] = data.getByte(startIndex + i);
      }
   }

   public static void main(String[] args)
   {
      try
      {
         if (args.length != 7)
         {
            System.out.println("Using the default parameters");
            new PointCloud2ToLidarScanMessageConverter(POINT_CLOUD_2_TOPIC_NAME,
                                                       SENSOR_POSE_X,
                                                       SENSOR_POSE_Y,
                                                       SENSOR_POSE_Z,
                                                       SENSOR_POSE_YAW,
                                                       SENSOR_POSE_PITCH,
                                                       SENSOR_POSE_ROLL);
         }
         else
         {
            System.out.println("Loading parameters from program arguments");
            String topicName = args[0];
            double poseX = Double.parseDouble(args[1]);
            double poseY = Double.parseDouble(args[2]);
            double poseZ = Double.parseDouble(args[3]);
            double poseYaw = Double.parseDouble(args[4]);
            double posePitch = Double.parseDouble(args[5]);
            double poseRoll = Double.parseDouble(args[6]);
            new PointCloud2ToLidarScanMessageConverter(topicName, poseX, poseY, poseZ, poseYaw, posePitch, poseRoll);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
