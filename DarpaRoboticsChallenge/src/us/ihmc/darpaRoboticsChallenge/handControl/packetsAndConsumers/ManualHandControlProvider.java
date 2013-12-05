package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.net.ObjectConsumer;

public class ManualHandControlProvider implements ObjectConsumer<ManualHandControlPacket>
{
   
   private final ConcurrentLinkedQueue<ManualHandControlPacket> packetQueue = new ConcurrentLinkedQueue<ManualHandControlPacket>();

   public void consumeObject(ManualHandControlPacket packet)
   {
      packetQueue.add(packet);
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
      if(!isNewPacketAvailable())
      {
         return null;
      }
      return packetQueue.peek().getSide();
   }
   
}

