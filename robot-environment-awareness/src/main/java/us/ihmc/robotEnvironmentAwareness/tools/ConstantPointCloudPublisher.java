package us.ihmc.robotEnvironmentAwareness.tools;

import controller_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

import java.util.Random;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicBoolean;

public class ConstantPointCloudPublisher
{
   public ConstantPointCloudPublisher()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
      IHMCROS2Publisher<LidarScanMessage> publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);

      LidarScanMessage lidarScanMessage = new LidarScanMessage();
      lidarScanMessage.getLidarPosition().set(0.0, 0.0, 0.7);
      lidarScanMessage.getLidarOrientation().setToPitchOrientation(0.5);

      packSquarePoints(lidarScanMessage.getScan());
//      packRandomPoints(lidarScanMessage.getScan());

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

   private static void packSquarePoints(IDLSequence.Float scanToPack)
   {
      float spacing = 0.0035f;
      int numPointsWidth = 80;

      for (int i = 0; i < numPointsWidth; i++)
      {
         for (int j = 0; j < numPointsWidth; j++)
         {
            scanToPack.add(0.4f + 0.2f * spacing * j);
            scanToPack.add((i - 0.5f * numPointsWidth) * spacing);
            scanToPack.add((j - 0.5f * numPointsWidth) * spacing + 0.6f);
         }
      }
   }

   private static void packRandomPoints(IDLSequence.Float scanToPack)
   {
      int numPoints = 8000;
      Random random = new Random(920);

      for (int i = 0; i < 3 * numPoints; i++)
      {
         float coord = (float) EuclidCoreRandomTools.nextDouble(random, 0.7);
         scanToPack.add(coord);
      }
   }

   public static void main(String[] args)
   {
      new ConstantPointCloudPublisher();
   }
}
