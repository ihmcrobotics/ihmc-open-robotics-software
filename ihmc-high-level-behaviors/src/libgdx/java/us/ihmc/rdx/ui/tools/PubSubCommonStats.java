package us.ihmc.rdx.ui.tools;

import static us.ihmc.jros2.Statistics.StatisticDataType.LATEST;
import static us.ihmc.jros2.Statistics.StatisticDataType.MAXIMUM;
import static us.ihmc.jros2.Statistics.StatisticDataType.SAMPLE_COUNT;
import static us.ihmc.jros2.Statistics.StatisticDataType.TOTAL;

import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.Statistics;

public abstract class PubSubCommonStats
{
   private final ROS2Node node;
   private final Statistics sizeStatistics = new Statistics();
   private final PubSubRateCalculator eventFrequencyCalculator = new PubSubRateCalculator();
   private final PubSubRateCalculator bandwidthCalculator = new PubSubRateCalculator();
   private double eventFrequency = 0.0;
   private double bandwidth = 0.0;

   protected PubSubCommonStats(ROS2Node node)
   {
      this.node = node;
   }

   protected abstract void readSizeStatistics(Statistics statisticsToPack);

   /** This should be called at a periodic rate to update the derivative calculations. */
   public void update()
   {
      readSizeStatistics(sizeStatistics);

      eventFrequency = eventFrequencyCalculator.finiteDifference(getSampleCount());
      bandwidth = bandwidthCalculator.finiteDifference(getTotalBytes());
   }

   public ROS2Node getNode()
   {
      return node;
   }

   public long getSampleCount()
   {
      return (long) sizeStatistics.get(SAMPLE_COUNT);
   }

   public long getTotalBytes()
   {
      return (long) sizeStatistics.get(TOTAL);
   }

   public long getLargestMessageSize()
   {
      return (long) sizeStatistics.get(MAXIMUM);
   }

   public long getCurrentMessageSize()
   {
      return (long) sizeStatistics.get(LATEST);
   }

   public double getEventFrequency()
   {
      return eventFrequency;
   }

   public double getBandwidth()
   {
      return bandwidth;
   }
}
