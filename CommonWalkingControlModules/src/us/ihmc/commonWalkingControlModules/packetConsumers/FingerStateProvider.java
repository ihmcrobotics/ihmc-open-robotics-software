package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class FingerStateProvider implements PacketConsumer<HandDesiredConfigurationMessage>
{
   private final ConcurrentLinkedQueue<HandDesiredConfigurationMessage> packetQueue = new ConcurrentLinkedQueue<HandDesiredConfigurationMessage>();
   private RobotSide robotSide;
   
   public FingerStateProvider(RobotSide robotSide)
   {
	   this.robotSide = robotSide;
   }
   
   public void receivedPacket(HandDesiredConfigurationMessage packet)
   {
	   if(this.robotSide == null)
	      packetQueue.add(packet);
	   else if(packet.getRobotSide() == this.robotSide)
		   packetQueue.add(packet);
   }

   public HandDesiredConfigurationMessage pullPacket()
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
