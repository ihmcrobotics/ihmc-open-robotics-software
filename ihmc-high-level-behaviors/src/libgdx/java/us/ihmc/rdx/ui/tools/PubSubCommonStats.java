package us.ihmc.rdx.ui.tools;

import us.ihmc.pubsub.participant.Participant;

public abstract class PubSubCommonStats
{
   /**
    * We have observed issues when messages are larger than this.
    */
   public static final int HIGH_PAYLOAD_LIMIT = 250000;

   private final Participant participant;

   // Analysis fields -- not modified by pubsub threads
   private final PubSubRateCalculator eventFrequencyCalculator = new PubSubRateCalculator();
   private final PubSubRateCalculator bandwidthCalculator = new PubSubRateCalculator();
   private double publishFrequency = 0.0;
   private double bandwidth = 0.0;

   public PubSubCommonStats(Participant participant)
   {
      this.participant = participant;
   }

   /** This should be called at a periodic rate to update the derivative calculations. */
   public void update()
   {
      publishFrequency = eventFrequencyCalculator.finiteDifference(getNumberOfEvents());
      bandwidth = bandwidthCalculator.finiteDifference(getCumulativePayloadBytes());
   }

   public abstract long getNumberOfEvents();

   public abstract long getCumulativePayloadBytes();

   public double getEventFrequency()
   {
      return publishFrequency;
   }

   public double getBandwidth()
   {
      return bandwidth;
   }

   public Participant getParticipant()
   {
      return participant;
   }
}
