package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;

public interface IncomingCommunicationBridgeInterface
{
   public void attachGlobalListener(GlobalPacketConsumer listener);
   public void detachGlobalListener(GlobalPacketConsumer listener);
}
