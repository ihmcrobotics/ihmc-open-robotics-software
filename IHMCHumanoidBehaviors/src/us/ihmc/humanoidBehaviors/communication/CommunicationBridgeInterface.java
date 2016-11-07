package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;

public interface CommunicationBridgeInterface 
{
	public void sendPacketToController(Packet obj);
	public void sendPacketToNetworkProcessor(Packet obj);
	public void sendPacketToUI(Packet obj);
	public void sendPacketToBehavior(Packet obj);
   public void attachGlobalListener(GlobalPacketConsumer listener);
   public void detachGlobalListener(GlobalPacketConsumer listener);
   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> listener);
   public <T extends Packet> void detachListener(Class<T> clazz, PacketConsumer<T> listener);


}
