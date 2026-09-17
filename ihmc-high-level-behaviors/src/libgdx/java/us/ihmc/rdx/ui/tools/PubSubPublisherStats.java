package us.ihmc.rdx.ui.tools;

import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.Statistics;

public class PubSubPublisherStats extends PubSubCommonStats
{
   private final ROS2Publisher<?> publisher;

   public PubSubPublisherStats(ROS2Node node, ROS2Publisher<?> publisher)
   {
      super(node);
      this.publisher = publisher;
   }

   @Override
   protected void readSizeStatistics(Statistics statisticsToPack)
   {
      publisher.readMessageSizeStatistics(statisticsToPack);
   }

   public ROS2Publisher<?> getPublisher()
   {
      return publisher;
   }

   public double getPublishFrequency()
   {
      return getEventFrequency();
   }
}
