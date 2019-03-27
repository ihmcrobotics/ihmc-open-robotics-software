package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import us.ihmc.commons.Conversions;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class TimingInformation
{
   private long previousTime = 0;
   private YoDouble actualDTYoDouble;
   private long avgJitter = 0;
   private long maxJitter = 0;

   private long periodInNS;
   private long iterations = 0;

   private boolean isInitialized = false;

   public TimingInformation(String name, long periodInNS)
   {
      this.periodInNS = periodInNS;

      System.out.println(name + " Period, Hz: " + 1 / (periodInNS / 1e9));
   }

   public void initialize(long currentTime, YoDouble actualDTYoDouble)
   {
      previousTime = currentTime;
      this.actualDTYoDouble = actualDTYoDouble;
      isInitialized = true;
   }

   public void updateTimingInformation(long newTime)
   {
      long jitter = Math.abs(newTime - previousTime - periodInNS);

      if (actualDTYoDouble != null)
      {
         actualDTYoDouble.set(Conversions.nanosecondsToMilliseconds((double) newTime - (double) previousTime));
      }

      if (jitter > maxJitter)
      {
         maxJitter = jitter;
      }

      previousTime = newTime;
      avgJitter += jitter;

      iterations++;
   }

   public double getFinalMaxJitterSeconds()
   {
      return (double) maxJitter / 1e9;
   }

   public double getFinalAvgJitterSeconds()
   {
      return (double) avgJitter / (double) iterations / 1e9;
   }

   public boolean isInitialized()
   {
      return isInitialized;
   }
}
