package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.GlobalObjectConsumer;

public interface IncomingCommunicationBridgeInterface
{
   public void attachGlobalListenerToNetworkProcessor(GlobalObjectConsumer listener);

   public void attachGlobalListenerToController(GlobalObjectConsumer listener);

   public void detachGlobalListenerFromController(GlobalObjectConsumer listener);

   public void detachGlobalListenerFromNetworkProcessor(GlobalObjectConsumer listener);
}
