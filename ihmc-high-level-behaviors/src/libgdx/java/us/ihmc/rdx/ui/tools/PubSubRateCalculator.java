package us.ihmc.rdx.ui.tools;

import us.ihmc.commons.Conversions;

public class PubSubRateCalculator
{
   private double lastTime = Conversions.millisecondsToSeconds(System.currentTimeMillis());
   private long lastValue = 0;
   private double lastRate;

   public double finiteDifference(long currentValue)
   {
      double currentTime = Conversions.millisecondsToSeconds(System.currentTimeMillis());

      long change = currentValue - lastValue;
      double dt = currentTime - lastTime;

      lastValue = currentValue;
      lastTime = currentTime;

      double currentRate = change / dt;

      double alpha = 0.99;
      currentRate = alpha * lastRate + (1.0 - alpha) * currentRate;

      lastRate = currentRate;

      return lastRate;
   }
}
