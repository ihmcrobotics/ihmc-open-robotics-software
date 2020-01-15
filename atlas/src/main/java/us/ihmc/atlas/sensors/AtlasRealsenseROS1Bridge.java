package us.ihmc.atlas.sensors;

import sensor_msgs.PointField;
import sensor_msgs.msg.dds.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.ConcurrentLinkedDeque;

/**
 * This class is to subscribe to the Intel Realsense ROS 1 topic and publish
 * a reduced PointCloud2 on ROS 2.
 *
 * TODO: Add T265
 */
public class AtlasRealsenseROS1Bridge
{
   private final RosMainNode rosMainNode;

   // create concurrent queue
   private final ConcurrentLinkedDeque<sensor_msgs.PointCloud2> ros1Deque = new ConcurrentLinkedDeque<>();
   private final IHMCROS2Publisher<PointCloud2> pointCloud2ROS2Publisher;

   public AtlasRealsenseROS1Bridge()
   {
      URI rosCoreURI = NetworkParameters.getROSURI();
      rosMainNode = new RosMainNode(rosCoreURI, "atlas/realsensebridge", true);

      String topicName = "/depthcam/depth/color/points";

      rosMainNode.attachSubscriber(topicName, new AbstractRosTopicSubscriber<sensor_msgs.PointCloud2>(sensor_msgs.PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(sensor_msgs.PointCloud2 pointCloud2)
         {
            // republish immediately or store and move on
            ros1Deque.addFirst(pointCloud2);
         }
      });

      rosMainNode.execute();
      while (!rosMainNode.isStarted())
      {
         LogTools.info("Waiting for " + rosMainNode.getDefaultNodeName() + " to start...");
         ThreadTools.sleep(2000);
      }

      Ros2Node ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_ros1_bridge");

      pointCloud2ROS2Publisher = new IHMCROS2Publisher<>(ros2Node, PointCloud2.class);

      new PausablePeriodicThread("PublishThread", UnitConversions.hertzToSeconds(8), this::dequeAndPublish);
   }

   private void dequeAndPublish()
   {
      while (ros1Deque.size() > 10)
      {
         ros1Deque.removeLast();
      }

      sensor_msgs.PointCloud2 ros1PointCloud2 = ros1Deque.removeLast();

      sensor_msgs.msg.dds.PointCloud2 ros2PointCloud2 = new PointCloud2();

      long timestamp = ros1PointCloud2.getHeader().getStamp().totalNsecs();

      ros2PointCloud2 = new sensor_msgs.msg.dds.PointCloud2();
      ros2PointCloud2.setWidth(ros1PointCloud2.getWidth());
      ros2PointCloud2.setHeight(ros1PointCloud2.getHeight());
      ros2PointCloud2.setIsBigendian(true);
      ros2PointCloud2.setIsDense(true);
      int pointStep = ros1PointCloud2.getPointStep();
      ros2PointCloud2.setPointStep(pointStep);
      ros2PointCloud2.setRowStep(ros1PointCloud2.getRowStep());

      int numberOfPoints = ros1PointCloud2.getWidth() * ros1PointCloud2.getHeight();
      int arrayOffset = ros1PointCloud2.getData().arrayOffset();
      ByteBuffer byteBuffer = ByteBuffer.wrap(ros1PointCloud2.getData().array(), arrayOffset, numberOfPoints * pointStep);

      if (ros1PointCloud2.getIsBigendian())
         byteBuffer.order(ByteOrder.BIG_ENDIAN);
      else
         byteBuffer.order(ByteOrder.LITTLE_ENDIAN);

      for (int i = 0; i < 100; i++)
      {
         PointField ros1PointField = ros1PointCloud2.getFields().get(i);
         sensor_msgs.msg.dds.PointField ros2PointField = new sensor_msgs.msg.dds.PointField();
         ros2PointField.setOffset(ros1PointField.getOffset());
         ros2PointField.setCount(ros1PointField.getCount());
         ros2PointField.setDatatype(ros1PointField.getDatatype());
         ros2PointCloud2.getFields().add(ros2PointField);

         byteBuffer.position(i * pointStep + arrayOffset);
         ros2PointCloud2.getData().add(byteBuffer.get());
         ros2PointCloud2.getData().add(byteBuffer.get());
         ros2PointCloud2.getData().add(byteBuffer.get());
      }

      pointCloud2ROS2Publisher.publish(ros2PointCloud2);
   }

   public static void main(String[] args)
   {
      new AtlasRealsenseROS1Bridge();
   }
}
