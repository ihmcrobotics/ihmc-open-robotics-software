package us.ihmc.robotics.time;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;

public class ExecutionTimer
{
   private final long measurementDelay;
   private long timeOfFirstMeasurement = Long.MAX_VALUE;

   private final DoubleYoVariable current;
   private final DoubleYoVariable average;
   private final DoubleYoVariable standardDeviation;
   private final DoubleYoVariable maximum;
   private final SimpleMovingAverageFilteredYoVariable movingAverage;
   private final LongYoVariable count;

   private long startTime;

   public ExecutionTimer(String name, double measurementDelayInSeconds, YoVariableRegistry registry)
   {
      this.measurementDelay = (long) (measurementDelayInSeconds * 1e9);

      current = new DoubleYoVariable(name + "Current", registry);
      average = new DoubleYoVariable(name + "Average", registry);
      movingAverage = new SimpleMovingAverageFilteredYoVariable(name + "MovingAverage", 100, current, registry);
      standardDeviation = new DoubleYoVariable(name + "StandardDeviation", registry);
      maximum = new DoubleYoVariable(name + "Maximum", registry);
      count = new LongYoVariable(name + "Count", registry);
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
         final double timeTaken = ((double) (currentNanoTime - startTime)) / 1e9;
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

   public DoubleYoVariable getCurrentTime()
   {
      return current;
   }

   public DoubleYoVariable getAverageTime()
   {
      return average;
   }

   public DoubleYoVariable getMovingAverage()
   {
      return movingAverage;
   }

   public DoubleYoVariable getStandardDeviation()
   {
      return standardDeviation;
   }

   public DoubleYoVariable getMaxTime()
   {
      return maximum;
   }
}
