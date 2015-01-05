package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.PacketConsumer;

public interface IncomingCommunicationBridgeInterface
{
   public void attachGlobalListenerToNetworkProcessor(PacketConsumer listener);

   public void attachGlobalListenerToController(PacketConsumer listener);

   public void detachGlobalListenerFromController(PacketConsumer listener);

   public void detachGlobalListenerFromNetworkProcessor(PacketConsumer listener);
}
