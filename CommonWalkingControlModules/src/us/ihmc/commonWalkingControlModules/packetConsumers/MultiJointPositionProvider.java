package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MultiJointAnglePacket;

public class MultiJointPositionProvider
{
	private final PacketConsumer<MultiJointAnglePacket> packetConsumer;
	private final AtomicReference<MultiJointAnglePacket> lastPacket = new AtomicReference<MultiJointAnglePacket>();
	
	public MultiJointPositionProvider()
	{
	   packetConsumer = new PacketConsumer<MultiJointAnglePacket>()
	   {
	      @Override
	      public void receivedPacket(MultiJointAnglePacket packet)
	      {     
	         if (packet != null)
	         {
	            lastPacket.set(packet);
	         }
	      }
	   };
	}
	
	public boolean checkForNewPacket()
	{
	   return lastPacket.get() != null;
	}
	
	public MultiJointAnglePacket getNewPacket()
	{
	   return lastPacket.getAndSet(null);
	}
	
	public PacketConsumer<MultiJointAnglePacket> getPacketConsumer()
	{
	   return packetConsumer;
	}
}
