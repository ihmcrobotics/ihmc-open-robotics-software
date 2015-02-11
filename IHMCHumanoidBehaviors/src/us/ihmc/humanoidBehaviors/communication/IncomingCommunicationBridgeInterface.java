package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.PacketConsumer;

public interface IncomingCommunicationBridgeInterface
{
   public void attachGlobalListener(PacketConsumer listener);
   public void detachGlobalListener(PacketConsumer listener);
}
