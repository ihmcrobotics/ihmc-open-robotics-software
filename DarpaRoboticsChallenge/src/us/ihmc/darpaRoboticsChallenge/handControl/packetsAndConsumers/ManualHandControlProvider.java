package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.utilities.net.ObjectConsumer;

public class ManualHandControlProvider implements ObjectConsumer<ManualHandControlPacket>
{
   
   private final ConcurrentLinkedQueue<ManualHandControlPacket> packetQueue = new ConcurrentLinkedQueue<ManualHandControlPacket>();

   public void consumeObject(ManualHandControlPacket packet)
   {
      packetQueue.add(packet);
   }

   public ManualHandControlPacket getPacket()
   {
      return packetQueue.poll();
   }

   public boolean isNewPacketAvailable()
   {
      return !packetQueue.isEmpty();
   }
   
}

