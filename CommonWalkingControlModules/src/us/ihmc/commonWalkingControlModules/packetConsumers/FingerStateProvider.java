package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.robotSide.RobotSide;

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
