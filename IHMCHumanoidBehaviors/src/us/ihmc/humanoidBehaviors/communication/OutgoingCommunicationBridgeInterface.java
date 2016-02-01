package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.packets.Packet;

public interface OutgoingCommunicationBridgeInterface 
{
	public void sendPacketToController(Packet obj);
	public void sendPacketToNetworkProcessor(Packet obj);
}
