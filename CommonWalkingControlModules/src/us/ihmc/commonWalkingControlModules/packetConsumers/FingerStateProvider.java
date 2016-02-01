package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.robotics.robotSide.RobotSide;

public class FingerStateProvider implements PacketConsumer<FingerStatePacket>
{
   private final ConcurrentLinkedQueue<FingerStatePacket> packetQueue = new ConcurrentLinkedQueue<FingerStatePacket>();
   private RobotSide robotSide;
   
   public FingerStateProvider(RobotSide robotSide)
   {
	   this.robotSide = robotSide;
   }
   
   public void receivedPacket(FingerStatePacket packet)
   {
	   if(this.robotSide == null)
	      packetQueue.add(packet);
	   else if(packet.getRobotSide() == this.robotSide)
		   packetQueue.add(packet);
   }

   public FingerStatePacket pullPacket()
   {
      return packetQueue.poll();
   }

   public boolean isNewFingerStateAvailable()
   {
      return !packetQueue.isEmpty();
   }
   
   public RobotSide getSide()
   {
	   return robotSide;
   }
}
