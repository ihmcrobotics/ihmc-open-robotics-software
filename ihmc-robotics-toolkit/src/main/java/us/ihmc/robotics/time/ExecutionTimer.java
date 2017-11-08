package us.ihmc.robotics.time;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;

public class ExecutionTimer
{
   private final long measurementDelay;
   private long timeOfFirstMeasurement = Long.MAX_VALUE;

   private final YoDouble current;
   private final YoDouble average;
   private final YoDouble standardDeviation;
   private final YoDouble maximum;
   private final SimpleMovingAverageFilteredYoVariable movingAverage;
   private final YoLong count;

   private long startTime;

   public ExecutionTimer(String name, YoVariableRegistry registry)
   {
      this(name, 0.0, registry);
   }
   
   public ExecutionTimer(String name, double measurementDelayInSeconds, YoVariableRegistry registry)
   {
      this.measurementDelay = Conversions.secondsToNanoseconds(measurementDelayInSeconds);

      current = new YoDouble(name + "Current", registry);
      average = new YoDouble(name + "Average", registry);
      movingAverage = new SimpleMovingAverageFilteredYoVariable(name + "MovingAverage", 100, current, registry);
      standardDeviation = new YoDouble(name + "StandardDeviation", registry);
      maximum = new YoDouble(name + "Maximum", registry);
      count = new YoLong(name + "Count", registry);
   }

   public void startMeasurement()
   {
      startTime = System.nanoTime();

      if (timeOfFirstMeasurement == Long.MAX_VALUE)
      {
         timeOfFirstMeasurement = startTime;
      }
   }

   public void stopMeasurement()
   {
      final long currentNanoTime = System.nanoTime();
      if ((currentNanoTime - timeOfFirstMeasurement) > measurementDelay)
      {
         final double timeTaken = Conversions.nanosecondsToSeconds(currentNanoTime - startTime);
         final double previousAverage = average.getDoubleValue();
         double previousSumOfSquares = MathTools.square(standardDeviation.getDoubleValue()) * ((double) count.getLongValue());

         count.increment();

         current.set(timeTaken);

         average.set(previousAverage + (timeTaken - previousAverage) / ((double) count.getLongValue()));
         movingAverage.update();

         double sumOfSquares = previousSumOfSquares + (timeTaken - previousAverage) * (timeTaken - average.getDoubleValue());

         standardDeviation.set(Math.sqrt(sumOfSquares / ((double) count.getLongValue())));

         if (timeTaken > maximum.getDoubleValue())
         {
            maximum.set(timeTaken);
         }
      }
   }

   public YoDouble getCurrentTime()
   {
      return current;
   }

   public YoDouble getAverageTime()
   {
      return average;
   }

   public YoDouble getMovingAverage()
   {
      return movingAverage;
   }

   public YoDouble getStandardDeviation()
   {
      return standardDeviation;
   }

   public YoDouble getMaxTime()
   {
      return maximum;
   }
}
