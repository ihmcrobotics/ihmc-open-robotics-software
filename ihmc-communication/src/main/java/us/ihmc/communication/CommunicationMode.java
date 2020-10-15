package us.ihmc.communication;

import us.ihmc.pubsub.DomainFactory;

/**
 * Expirimental class to provide an app setting for inter or intra process mode
 * when the technologies used is more than just ROS 2 (PubSub).
 */
public enum CommunicationMode
{
   INTERPROCESS(DomainFactory.PubSubImplementation.FAST_RTPS),
   INTRAPROCESS(DomainFactory.PubSubImplementation.INTRAPROCESS);

   private DomainFactory.PubSubImplementation pubSubImplementation;

   CommunicationMode(DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this.pubSubImplementation = pubSubImplementation;
   }

   public DomainFactory.PubSubImplementation getPubSubImplementation()
   {
      return pubSubImplementation;
   }
}
