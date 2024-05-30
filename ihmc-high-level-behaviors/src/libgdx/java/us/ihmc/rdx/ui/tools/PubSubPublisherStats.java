package us.ihmc.rdx.ui.tools;

import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;

public class PubSubPublisherStats extends PubSubCommonStats
{
   private final Publisher publisher;

   public PubSubPublisherStats(Participant participant, Publisher publisher)
   {
      super(participant);

      this.publisher = publisher;
   }

   public Publisher getPublisher()
   {
      return publisher;
   }

   @Override
   public long getNumberOfEvents()
   {
      return publisher.getNumberOfPublications();
   }

   @Override
   public long getCumulativePayloadBytes()
   {
      return publisher.getCumulativePayloadBytes();
   }

   public double getPublishFrequency()
   {
      return getEventFrequency();
   }
}