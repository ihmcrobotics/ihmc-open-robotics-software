package us.ihmc.rdx.ui.tools;

import us.ihmc.commons.Conversions;

public class PubSubRateCalculator
{
   private double lastTime = Conversions.millisecondsToSeconds(System.currentTimeMillis());
   private long lastValue = 0;
   private double lastRate = 0.0;

   public double finiteDifference(long currentValue)
   {
      if (Double.isNaN(lastRate) || Double.isInfinite(lastRate))
         lastRate = 0.0;

      double currentTime = Conversions.millisecondsToSeconds(System.currentTimeMillis());

      long change = currentValue - lastValue;
      double dt = currentTime - lastTime;

      double currentRate = change / dt;

      lastValue = currentValue;
      lastTime = currentTime;

      double alpha = 0.99;
      currentRate = alpha * lastRate + (1.0 - alpha) * currentRate;

      lastRate = currentRate;

      return lastRate;
   }
}
