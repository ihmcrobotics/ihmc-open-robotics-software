package us.ihmc.avatar.lidar;

import controller_msgs.msg.dds.BigVideoPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

public class OusterLidarOnRobotProcess
{
   private final NettyOuster nettyOuster;
   private final ROS2Helper ros2Helper;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<BigVideoPacket> ros2VideoPublisher;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final ROS2SyncedRobotModel syncedRobot;

   public OusterLidarOnRobotProcess(DRCRobotModel robotModel)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_node");
      ros2Helper = new ROS2Helper(ros2Node);

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_lidar_publisher");

      ROS2Topic<BigVideoPacket> depthVideoTopic = ROS2Tools.OUSTER_LIDAR;
      LogTools.info("Publishing ROS 2 ouster lidar depth video: {}", depthVideoTopic);
      ros2VideoPublisher = ROS2Tools.createPublisher(realtimeROS2Node, depthVideoTopic, ROS2QosProfile.BEST_EFFORT());

      nettyOuster = new NettyOuster();
      nettyOuster.setOnFrameReceived(() ->
      {
         nettyOuster.getDepthImageMeters().rewind();
         byte[] heapByteArrayData = new byte[nettyOuster.getDepthImageMeters().getBackingDirectByteBuffer().remaining()];
         nettyOuster.getDepthImageMeters().getBackingDirectByteBuffer().get(heapByteArrayData);

         videoPacket.getData().resetQuick();
         videoPacket.getData().add(heapByteArrayData);

         videoPacket.setImageHeight(nettyOuster.getImageHeight());
         videoPacket.setImageWidth(nettyOuster.getImageWidth());
         videoPacket.setAcquisitionTimeSecondsSinceEpoch(nettyOuster.getAquisitionInstant().getEpochSecond());
         videoPacket.setAcquisitionTimeAdditionalNanos(nettyOuster.getAquisitionInstant().getNano());

         ros2VideoPublisher.publish(videoPacket);
      });
      nettyOuster.bind();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "DualBlackflyShutdown"));
   }

   private void destroy()
   {
      nettyOuster.destroy();
   }
}
