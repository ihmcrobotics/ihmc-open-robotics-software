package us.ihmc.perception.filters;

import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.tools.time.DurationCalculator;

public class BreakFrequencyAlphaCalculator
{
   private final int DT_AVERAGING_HISTORY = 3;
   private boolean firstRun = true;
   private final DurationCalculator periodCalculator = new DurationCalculator(DT_AVERAGING_HISTORY);

   public double calculateAlpha(double breakFrequency)
   {
      if (firstRun)
      {
         firstRun = false;
         periodCalculator.reset();
         return 0.0;
      }
      else
      {
         periodCalculator.ping();
         double averagePeriod = periodCalculator.getDuration();
         return AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, averagePeriod);
      }
   }
}
