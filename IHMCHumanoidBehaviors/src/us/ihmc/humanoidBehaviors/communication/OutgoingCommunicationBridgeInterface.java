package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.packets.Packet;

public interface OutgoingCommunicationBridgeInterface 
{
	public void sendPacketToController(Packet obj);
	public void sendPacketToNetworkProcessor(Packet obj);
	public void sendPacketToUI(Packet obj);
	public void sendPacketToBehavior(Packet obj);


}
