package us.ihmc.rdx.ui.tools;

import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.Statistics;

public class PubSubSubscriberStats extends PubSubCommonStats
{
   private final ROS2Subscription<?> subscription;

   public PubSubSubscriberStats(ROS2Node node, ROS2Subscription<?> subscription)
   {
      super(node);
      this.subscription = subscription;
   }

   @Override
   protected void readSizeStatistics(Statistics statisticsToPack)
   {
      subscription.readMessageSizeStatistics(statisticsToPack);
   }

   public ROS2Subscription<?> getSubscription()
   {
      return subscription;
   }

   public double getReceiveFrequency()
   {
      return getEventFrequency();
   }
}
