package us.ihmc.avatar.lidar;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class OusterDepthPublisher
{
   private final NettyOuster nettyOuster;
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_lidar_publisher_node");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
   private final VideoPacket videoPacket = new VideoPacket();

   public OusterDepthPublisher()
   {
      ROS2Topic<VideoPacket> depthVideoTopic = ROS2Tools.OUSTER_DEPTH;
      LogTools.info("Publishing ROS 2 ouster lidar depth video: {}", depthVideoTopic);

      nettyOuster = new NettyOuster();
      nettyOuster.setOnFrameReceived(() ->
      {
          nettyOuster.getDepthImageMeters().rewind();

         BytePointer compressedDepthPointer = new BytePointer();

         BytedecoOpenCVTools.compressImagePNG(nettyOuster.getDepthImageMeters().getBytedecoOpenCVMat(), compressedDepthPointer);

         byte[] heapByteArrayData = new byte[compressedDepthPointer.asBuffer().remaining()];
         compressedDepthPointer.asBuffer().get(heapByteArrayData);
         videoPacket.getData().resetQuick();
         videoPacket.getData().add(heapByteArrayData);

         LogTools.info("Compressed Bytes: {}", heapByteArrayData.length);

         videoPacket.setImageHeight(nettyOuster.getImageHeight());
         videoPacket.setImageWidth(nettyOuster.getImageWidth());
         videoPacket.setTimestamp(nettyOuster.getAquisitionInstant().getNano());
         videoPacket.setVideoSource(VideoSource.MULTISENSE_LEFT_EYE.toByte());
//         videoPacket.setTimestamp(nettyOuster.getAquisitionInstant().getEpochSecond());

          LogTools.info("Publishing Ouster Depth: {} {} {}", videoPacket.getTimestamp(), videoPacket.getImageHeight(), videoPacket.getImageWidth());
         ros2Helper.publish(ROS2Tools.OUSTER_DEPTH, videoPacket);
      });
      nettyOuster.bind();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "OusterDepthPublisherShutdown"));
   }

   private void destroy()
   {
      nettyOuster.destroy();
   }
}
