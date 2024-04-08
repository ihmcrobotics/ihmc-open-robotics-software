package us.ihmc.perception.ros1.camera;

import boofcv.struct.calib.CameraPinholeBrown;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

public class VideoPacketHandler implements CompressedVideoHandler
{
   private final ROS2PublisherBasics<VideoPacket> publisher;

   private volatile boolean enable = true;

   public VideoPacketHandler(ROS2NodeInterface ros2Node)
   {
      this(ros2Node, PerceptionAPI.VIDEO);
   }

   public VideoPacketHandler(ROS2NodeInterface ros2Node, ROS2QosProfile qosProfile)
   {
      this(ros2Node, PerceptionAPI.VIDEO, qosProfile);
   }

   public VideoPacketHandler(ROS2NodeInterface ros2Node, ROS2Topic<VideoPacket> topic)
   {
      this(ros2Node, topic, ROS2QosProfile.RELIABLE());
   }

   public VideoPacketHandler(ROS2NodeInterface ros2Node, ROS2Topic<VideoPacket> topic, ROS2QosProfile qosProfile)
   {
      LogTools.info("Creating video publisher on topic: {}", topic.getName());
      publisher = ros2Node.createPublisher(topic);
   }

   private Stopwatch timer;
   {
      timer = new Stopwatch().start();
   }

   @Override
   public void onFrame(VideoSource videoSource,
                       byte[] data,
                       long timeStamp,
                       Point3DReadOnly position,
                       QuaternionReadOnly orientation,
                       CameraPinholeBrown intrinsicParameters)
   {
      LogTools.debug(() ->
      {
         double fps = 1.0 / timer.averageLap();
         timer.lap();
         return "Sending new VideoPacket FPS: " + fps;
      });

      if (!enable)
         return;

      VideoPacket message = HumanoidMessageTools.createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters);

      if (!enable)
         return;

      publisher.publish(message);
   }

   @Override
   public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
   {
   }

   @Override
   public boolean isConnected()
   {
      return true;
   }

   public void closeAndDispose()
   {
      enable = false;
   }
}