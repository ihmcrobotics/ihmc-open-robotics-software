package us.ihmc.rdx.ui.tools;

import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.subscriber.Subscriber;

public class ROS2SubscriberStats extends ROS2PubSubCommonStats
{
   private final Subscriber<?> subscriber;

   public ROS2SubscriberStats(Participant participant, Subscriber<?> subscriber)
   {
      super(participant);

      this.subscriber = subscriber;
   }

   public Subscriber<?> getSubscriber()
   {
      return subscriber;
   }

   @Override
   public long getNumberOfEvents()
   {
      return subscriber.getNumberOfReceivedMessages();
   }

   @Override
   public long getCumulativePayloadBytes()
   {
      return subscriber.getCumulativePayloadBytes();
   }

   public double getReceiveFrequency()
   {
      return getEventFrequency();
   }
}