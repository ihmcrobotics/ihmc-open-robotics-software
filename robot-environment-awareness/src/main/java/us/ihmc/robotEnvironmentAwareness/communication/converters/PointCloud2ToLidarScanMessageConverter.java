package us.ihmc.robotEnvironmentAwareness.communication.converters;

import controller_msgs.msg.dds.LidarScanMessage;
import gnu.trove.list.array.TByteArrayList;
import sensor_msgs.msg.dds.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

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
   private static final String POINT_CLOUD_2_TOPIC_NAME = "/os1_cloud_node/points";

   private static final double SENSOR_POSE_X = 0.0;
   private static final double SENSOR_POSE_Y = 0.0;
   private static final double SENSOR_POSE_Z = 1.0;
   private static final double SENSOR_POSE_YAW = 0.0;
   private static final double SENSOR_POSE_PITCH = 0.0;
   private static final double SENSOR_POSE_ROLL = 0.0;

   private final AtomicReference<PointCloud2> pointCloud2Message = new AtomicReference<>();
   private final AtomicInteger counter = new AtomicInteger();
   private final AtomicBoolean hasPrintedReceivedMessage = new AtomicBoolean();

   public PointCloud2ToLidarScanMessageConverter(String topicName, double poseX, double poseY, double poseZ, double poseYaw, double posePitch, double poseRoll)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
      Pose3D sensorPose = new Pose3D(poseX, poseY, poseZ, poseYaw, posePitch, poseRoll);

      IHMCROS2Publisher<LidarScanMessage> lidarScanMessagePublisher = ROS2Tools.createPublisher(ros2Node, LidarScanMessage.class, "/ihmc/lidar_scan");

      // save and handle on another thread
      ROS2Tools.createCallbackSubscription(ros2Node, PointCloud2.class, topicName, s -> pointCloud2Message.set(s.takeNextData()));

      LidarScanMessage lidarScanMessage = new LidarScanMessage();
      lidarScanMessage.getLidarPosition().set(sensorPose.getPosition());
      lidarScanMessage.getLidarOrientation().set(sensorPose.getOrientation());

      new Thread(() ->
                 {
                    while (true)
                    {
                       PointCloud2 pointCloud = this.pointCloud2Message.getAndSet(null);

                       if (pointCloud != null)
                       {
                          ByteOrder byteOrder = pointCloud.getIsBigendian() ? ByteOrder.BIG_ENDIAN : ByteOrder.LITTLE_ENDIAN;
                          long pointStep = pointCloud.getPointStep();
                          int width = (int) pointCloud.getWidth();
                          int height = (int) pointCloud.getHeight();
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

                             points[3 * i + 0] = x;
                             points[3 * i + 1] = y;
                             points[3 * i + 2] = z;
                          }

                          lidarScanMessage.setSequenceId(counter.getAndIncrement());
                          lidarScanMessage.getScan().clear();
                          lidarScanMessage.getScan().add(points);
                          lidarScanMessagePublisher.publish(lidarScanMessage);
                       }

                       if (!hasPrintedReceivedMessage.getAndSet(true))
                       {
                          LogTools.info("Received PointCloud2 message");
                       }

                       ThreadTools.sleep(20);
                    }
                 }).start();
   }

   private static void packBytes(byte[] bytes, int startIndex, TByteArrayList data)
   {
      for (int i = 0; i < 4; i++)
      {
         bytes[i] = data.get(startIndex + i);
      }
   }

   public static void main(String[] args)
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
}
