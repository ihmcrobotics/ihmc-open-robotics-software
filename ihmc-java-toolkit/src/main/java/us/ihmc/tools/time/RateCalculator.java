package us.ihmc.tools.time;

import us.ihmc.commons.Conversions;

import java.util.ArrayDeque;

public class RateCalculator
{
   private double lastTime = Conversions.millisecondsToSeconds(System.currentTimeMillis());
   private long lastValue = 0;

   private final ArrayDeque<Double> rates = new ArrayDeque<>();
   private final int windowSize;

   public RateCalculator(int windowSize)
   {
      this.windowSize = windowSize;
   }

   public double finiteDifference(long currentValue)
   {
      long change = currentValue - lastValue;

      double currentTime = Conversions.millisecondsToSeconds(System.currentTimeMillis());
      double dt = currentTime - lastTime;

      double newRate = change / dt;

      lastValue = currentValue;
      lastTime = currentTime;

      rates.addLast(newRate);

      while (rates.size() > windowSize)
      {
         rates.removeFirst();
      }

      double averageRate = 0.0;
      for (Double rate : rates)
      {
         averageRate += rate;
      }
      averageRate = averageRate / rates.size();

      return averageRate;
   }
}
