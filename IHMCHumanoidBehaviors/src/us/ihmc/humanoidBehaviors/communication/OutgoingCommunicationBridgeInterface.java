package us.ihmc.humanoidBehaviors.communication;

public interface OutgoingCommunicationBridgeInterface 
{
	public void sendPacketToController(Object obj);
	public void sendPacketToNetworkProcessor(Object obj);
}
