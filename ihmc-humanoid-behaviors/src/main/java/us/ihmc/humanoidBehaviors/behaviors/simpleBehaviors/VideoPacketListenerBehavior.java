package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.communication.video.VideoCallback;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ros2.Ros2Node;

public abstract class VideoPacketListenerBehavior extends AbstractBehavior implements VideoCallback
{
   private final ConcurrentListeningQueue<VideoPacket> cameraData = new ConcurrentListeningQueue<>(20);

   private final CompressedVideoDataClient videoDataClient;

   public VideoPacketListenerBehavior(String robotName, String namePrefix, Ros2Node ros2Node)
   {
      super(robotName, namePrefix, ros2Node);

      videoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);

      createSubscriber(VideoPacket.class, ROS2Tools.IHMC_ROOT, cameraData::put);
   }

   @Override
   public void doControl()
   {
      if (cameraData.isNewPacketAvailable())
      {
         VideoPacket packet = cameraData.poll();
         videoDataClient.onFrame(VideoSource.fromByte(packet.getVideoSource()), packet.getData().toArray(), packet.getTimestamp(), packet.getPosition(),
                                 packet.getOrientation(), HumanoidMessageTools.toIntrinsicParameters(packet.getIntrinsicParameters()));
      }
   }
}
