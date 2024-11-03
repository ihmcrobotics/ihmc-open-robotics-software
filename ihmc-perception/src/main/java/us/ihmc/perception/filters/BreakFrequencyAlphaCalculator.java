package us.ihmc.perception.filters;

import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.tools.time.MovingAverageDurationCalculator;

public class BreakFrequencyAlphaCalculator
{
   private final int DT_WINDOW_SIZE = 3;
   private boolean firstRun = true;
   private final MovingAverageDurationCalculator periodCalculator = new MovingAverageDurationCalculator(DT_WINDOW_SIZE);

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
