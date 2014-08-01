package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.packets.FingerStatePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.net.ObjectConsumer;

public class FingerStateProvider implements ObjectConsumer<FingerStatePacket>
{
   private final ConcurrentLinkedQueue<FingerStatePacket> packetQueue = new ConcurrentLinkedQueue<FingerStatePacket>();
   private RobotSide robotSide;
   
   public FingerStateProvider(RobotSide robotSide)
   {
	   this.robotSide = robotSide;
   }
   
   public void consumeObject(FingerStatePacket packet)
   {
	   if(packet.getRobotSide() == this.robotSide)
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
