package us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.net.ObjectConsumer;

public class ManualHandControlProvider implements ObjectConsumer<ManualHandControlPacket>
{
   
   private final ConcurrentLinkedQueue<ManualHandControlPacket> packetQueue = new ConcurrentLinkedQueue<ManualHandControlPacket>();
   private RobotSide robotSide;
   
   public ManualHandControlProvider(RobotSide robotSide)
   {
	   this.robotSide = robotSide;
   }
   
   public void consumeObject(ManualHandControlPacket packet)
   {
      if(packet.getRobotSide() == this.robotSide)
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
      return this.robotSide;
   }
   
}

