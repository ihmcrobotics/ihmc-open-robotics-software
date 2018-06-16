package us.ihmc.avatar.handControl.packetsAndConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.ManualHandControlPacket;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;

public class ManualHandControlProvider implements NewMessageListener<ManualHandControlPacket>
{

   private final ConcurrentLinkedQueue<ManualHandControlPacket> packetQueue = new ConcurrentLinkedQueue<ManualHandControlPacket>();
   private RobotSide robotSide;

   public ManualHandControlProvider(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void onNewDataMessage(Subscriber<ManualHandControlPacket> subscriber)
   {
      packetQueue.add(subscriber.takeNextData());
   }

   public ManualHandControlPacket pullPacket()
   {
      return packetQueue.poll();
   }

   public boolean isNewPacketAvailable()
   {
      return !packetQueue.isEmpty();
   }

   public RobotSide getSide()
   {
      return this.robotSide;
   }
}
