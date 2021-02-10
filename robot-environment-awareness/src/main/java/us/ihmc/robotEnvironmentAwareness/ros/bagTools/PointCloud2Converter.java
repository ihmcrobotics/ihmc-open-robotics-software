package us.ihmc.robotEnvironmentAwareness.ros.bagTools;

import controller_msgs.msg.dds.LidarScanMessage;
import gnu.trove.list.array.TByteArrayList;
import sensor_msgs.msg.dds.PointCloud2;
import sensor_msgs.msg.dds.PointCloud2PubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Converts incoming PointCloud2 messages to LidarScanMessage. The result is exported as a JSON file
 */
public class PointCloud2Converter
{
   private static final int maximumMessagesToSave = 1500;
   private static final double timeWithoutMessageToTerminateMillis = 5000;

   private static final String pointCloudTopicName = "/points"; // "/slam/odom/cloud";

   private static final int pointCloud2BufferSize = 1100000;
   private static int pointStep = 63;

   public PointCloud2Converter() throws IOException
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());

      AtomicLong timestamp = new AtomicLong(-1);
      AtomicInteger counter = new AtomicInteger();
      AtomicBoolean readFlag = new AtomicBoolean(true);
      List<PointCloud2> receivedMessages = new ArrayList<>();

      PointCloud2PubSubType pointCloudDataType = new PointCloud2PubSubType();
      NewMessageListener<PointCloud2> pointCloudCallback = s ->
      {
         if (readFlag.get())
         {
            receivedMessages.add(s.takeNextData());
            counter.incrementAndGet();
            timestamp.set(System.currentTimeMillis());
         }
      };

      ROS2QosProfile rosQoSProfile = ROS2QosProfile.DEFAULT();
      ros2Node.createSubscription(pointCloudDataType, pointCloudCallback, pointCloudTopicName, rosQoSProfile);

      ThreadFactory threadFactory = ThreadTools.createNamedThreadFactory(getClass().getSimpleName());
      ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1, threadFactory);
      List<LidarScanMessage> messageList = new ArrayList<>();

      System.out.println("Waiting for messages...");

      Runnable ihmcPointCloudRunnable = () ->
      {
         boolean terminate = counter.get() >= maximumMessagesToSave || (timestamp.get() != -1
                                                                && System.currentTimeMillis() - timestamp.get() > timeWithoutMessageToTerminateMillis);
         if (!terminate)
         {
            return;
         }

         readFlag.set(false);
         System.out.println("Processing " + receivedMessages.size() + " messages. counter = " + counter.get());

         for (int i = 0; i < receivedMessages.size(); i++)
         {
            PointCloud2 pointCloud = receivedMessages.get(i);
            int width = (int) pointCloud.getWidth();
            int height = (int) pointCloud.getHeight();
            int numberOfPoints = width * height;
            int numPointsToConsider = Math.min((pointCloud2BufferSize / pointStep) - 1, numberOfPoints);
            System.out.println("\t received " + numberOfPoints + ", processing " + numPointsToConsider);

            float[] points = new float[3 * numPointsToConsider];
            byte[] bytes = new byte[4];

            for (int j = 0; j < numPointsToConsider; j++)
            {
               int startIndex = pointStep * j;
               packBytes(bytes, startIndex + 0, pointCloud.getData());
               float x = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
               packBytes(bytes, startIndex + 4, pointCloud.getData());
               float y = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();
               packBytes(bytes, startIndex + 8, pointCloud.getData());
               float z = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getFloat();

               points[3 * j + 0] = x;
               points[3 * j + 1] = y;
               points[3 * j + 2] = z;
            }

            LidarScanMessage message = new LidarScanMessage();
            message.setSequenceId(counter.getAndIncrement());
            message.getScan().add(points);
            messageList.add(message);
         }

         LidarScanMessageExporter.export(messageList);
      };

      executorService.scheduleAtFixedRate(ihmcPointCloudRunnable, 0, 1, TimeUnit.MILLISECONDS);
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
