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

   public static final CommunicationMode[] VALUES = values();
   public static final String[] ROS2_NAMES = new String[] {"Fast-RTPS", "Intraprocess"};
   public static final String[] MESSAGER_NAMES = new String[] {"Kryonet", "Shared memory"};

   private DomainFactory.PubSubImplementation pubSubImplementation;

   CommunicationMode(DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this.pubSubImplementation = pubSubImplementation;
   }

   public DomainFactory.PubSubImplementation getPubSubImplementation()
   {
      return pubSubImplementation;
   }

   public static CommunicationMode fromOrdinal(int ordinal)
   {
      return VALUES[ordinal];
   }
}
