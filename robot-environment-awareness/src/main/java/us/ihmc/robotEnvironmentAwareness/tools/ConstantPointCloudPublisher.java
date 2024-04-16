package us.ihmc.robotEnvironmentAwareness.tools;

import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicBoolean;

public class ConstantPointCloudPublisher
{
   public ConstantPointCloudPublisher()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
      ROS2PublisherBasics<LidarScanMessage> publisher = ros2Node.createPublisher(PerceptionAPI.MULTISENSE_LIDAR_SCAN);

      LidarScanMessage lidarScanMessage = new LidarScanMessage();
      lidarScanMessage.getLidarPosition().set(0.0, 0.0, 0.7);
      lidarScanMessage.getLidarOrientation().setToPitchOrientation(0.5);

      packSquarePoints(lidarScanMessage);
//      packRandomPoints(lidarScanMessage);

      AtomicBoolean stop = new AtomicBoolean();
      Scanner keyboard = new Scanner(System.in);

      LogTools.info("Starting to publish pointcloud. Enter a letter followed by enter to cancel...");

      new Thread(() ->
                 {
                    while (!stop.get())
                    {
                       ThreadTools.sleep(300);
                       lidarScanMessage.setRobotTimestamp(System.nanoTime());
                       lidarScanMessage.setSequenceId(lidarScanMessage.getSequenceId() + 1);
                       publisher.publish(lidarScanMessage);
                    }

                    ros2Node.destroy();
                 }).start();

      new Thread(() ->
                 {
                    keyboard.next();
                    stop.set(true);
                 }).start();
   }

   private static void packSquarePoints(LidarScanMessage lidarScanMessage)
   {
      float spacing = 0.0035f;
      int numPointsWidth = 80;

      List<Point3D> points = new ArrayList<>();
      for (int i = 0; i < numPointsWidth; i++)
      {
         for (int j = 0; j < numPointsWidth; j++)
         {
            double x = (0.4f + 0.2f * spacing * j);
            double y = ((i - 0.5f * numPointsWidth) * spacing);
            double z = ((j - 0.5f * numPointsWidth) * spacing + 0.6f);
            points.add(new Point3D(x, y, z));
         }
      }

      LidarPointCloudCompression.compressPointCloud(points.size(), lidarScanMessage, (i, j) -> points.get(i).getElement32(j));
   }

   private static void packRandomPoints(LidarScanMessage lidarScanMessage)
   {
      int numPoints = 8000;
      Random random = new Random(920);

      List<Point3D> points = new ArrayList<>();
      for (int i = 0; i < 3 * numPoints; i++)
      {
         double x = (float) EuclidCoreRandomTools.nextDouble(random, 0.7);
         double y = (float) EuclidCoreRandomTools.nextDouble(random, 0.7);
         double z = (float) EuclidCoreRandomTools.nextDouble(random, 0.7);
         points.add(new Point3D(x, y, z));
      }

      LidarPointCloudCompression.compressPointCloud(points.size(), lidarScanMessage, (i, j) -> points.get(i).getElement32(j));
   }

   public static void main(String[] args)
   {
      new ConstantPointCloudPublisher();
   }
}
