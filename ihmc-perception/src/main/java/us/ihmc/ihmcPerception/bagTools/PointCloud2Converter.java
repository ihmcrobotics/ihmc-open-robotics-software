package us.ihmc.ihmcPerception.bagTools;

import controller_msgs.msg.dds.LidarScanMessage;
import geometry_msgs.msg.dds.PoseStamped;
import geometry_msgs.msg.dds.PoseStampedPubSubType;
import gnu.trove.list.array.TByteArrayList;
import org.apache.commons.lang3.mutable.MutableInt;
import sensor_msgs.msg.dds.PointCloud2;
import sensor_msgs.msg.dds.PointCloud2PubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2QosProfile;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Converts incoming PointCloud2 messages to LidarScanMessage. The result is exported as a JSON file
 */
public class PointCloud2Converter
{
   private static final int maximumMessagesToSave = 1500;
   private static final double timeWithoutMessageToTerminateMillis = 10000;

   private static final String pointCloudTopicName = "/points"; // "/slam/odom/cloud";
   private static final String poseTopicName = "/slam/odom/pose";

   public PointCloud2Converter() throws IOException
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      AtomicReference<PointCloud2> pointCloudReference = new AtomicReference<>();
      AtomicReference<PoseStamped> poseStampedReference = new AtomicReference<>(new PoseStamped());

      MutableInt counter = new MutableInt();
      AtomicLong previousTimestamp = new AtomicLong(-1);

      PointCloud2PubSubType pointCloudDataType = new PointCloud2PubSubType();
      NewMessageListener<PointCloud2> pointCloudCallback = s ->
      {
         PointCloud2 pointCloud2 = s.readNextData();
         pointCloudReference.set(pointCloud2);
      };

      PoseStampedPubSubType poseDataType = new PoseStampedPubSubType();
      NewMessageListener<PoseStamped> poseCallback = s ->
      {
         PoseStamped poseMessage = s.readNextData();
         poseStampedReference.set(poseMessage);
      };

      Ros2QosProfile rosQoSProfile = Ros2QosProfile.DEFAULT();
      ros2Node.createSubscription(pointCloudDataType, pointCloudCallback, pointCloudTopicName, rosQoSProfile);
      ros2Node.createSubscription(poseDataType, poseCallback, poseTopicName, rosQoSProfile);

      ThreadFactory threadFactory = ThreadTools.createNamedThreadFactory(getClass().getSimpleName());
      ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1, threadFactory);
      List<LidarScanMessage> messageList = new ArrayList<>();

      Runnable ihmcPointCloudRunnable = () ->
      {
         long timestamp = System.currentTimeMillis();
         if (previousTimestamp.get() == -1)
            previousTimestamp.set(timestamp);

         long dt = timestamp - previousTimestamp.get();
         if (dt > timeWithoutMessageToTerminateMillis)
         {
            LidarScanMessageExporter.export(messageList);
         }

         if (poseStampedReference.get() == null || pointCloudReference.get() == null)
         {
            return;
         }

         previousTimestamp.set(timestamp);

         PointCloud2 pointCloud = pointCloudReference.getAndSet(null);
         PoseStamped pose = poseStampedReference.get();

         int width = (int) pointCloud.getWidth();
         int height = (int) pointCloud.getHeight();
         int numberOfPoints = width * height;
         int numPointsToConsider = Math.min((100 / 26) - 1, numberOfPoints);

         float[] points = new float[3 * numPointsToConsider];
         byte[] bytes = new byte[4];

         for (int i = 0; i < numPointsToConsider; i++)
         {
            int startIndex = 26 * i;
            packBytes(bytes, startIndex + 0, pointCloud.getData());
            float x = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            packBytes(bytes, startIndex + 4, pointCloud.getData());
            float y = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
            packBytes(bytes, startIndex + 8, pointCloud.getData());
            float z = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();

            points[3 * i + 0] = x;
            points[3 * i + 1] = y;
            points[3 * i + 2] = z;
         }

         LidarScanMessage message = new LidarScanMessage();
         message.getLidarPosition().set(pose.getPose().getPosition());
         message.getLidarOrientation().set(pose.getPose().getOrientation());
         message.setSequenceId(counter.getAndIncrement());
         message.getScan().add(points);
         messageList.add(message);

         if (counter.getValue() >= maximumMessagesToSave)
         {
            LidarScanMessageExporter.export(messageList);
         }
      };

      executorService.scheduleAtFixedRate(ihmcPointCloudRunnable, 0, 5, TimeUnit.MILLISECONDS);
   }

   static void packBytes(byte[] bytes, int startIndex, TByteArrayList data)
   {
      for (int i = 0; i < 4; i++)
      {
         bytes[i] = data.get(startIndex + i);
      }
   }

   public static void main(String[] args) throws IOException
   {
      new PointCloud2Converter();
   }
}
