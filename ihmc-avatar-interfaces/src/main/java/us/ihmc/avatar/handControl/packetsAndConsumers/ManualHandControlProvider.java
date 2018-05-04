package us.ihmc.avatar.handControl.packetsAndConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.ManualHandControlPacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.robotics.robotSide.RobotSide;

public class ManualHandControlProvider implements PacketConsumer<ManualHandControlPacket>
{
   
   private final ConcurrentLinkedQueue<ManualHandControlPacket> packetQueue = new ConcurrentLinkedQueue<ManualHandControlPacket>();
   private RobotSide robotSide;
   
   public ManualHandControlProvider(RobotSide robotSide)
   {
	   this.robotSide = robotSide;
   }
   
   public void receivedPacket(ManualHandControlPacket packet)
   {
      if(packet.getRobotSide() == this.robotSide.toByte())
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

