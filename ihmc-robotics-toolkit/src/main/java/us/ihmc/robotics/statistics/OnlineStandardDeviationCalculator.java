package us.ihmc.robotics.statistics;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the Welford's online algorithm
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class OnlineStandardDeviationCalculator
{
   private final YoDouble variance;
   private final YoDouble populationVariance;
   private final YoDouble standardDeviation;
   private final YoDouble mean;
   private final YoDouble sumOfSquare;
   private final YoInteger numberOfSamples;

   private final DoubleProvider variable;

   public OnlineStandardDeviationCalculator(String prefix, YoRegistry registry)
   {
      this(prefix, null, registry);
   }

   public OnlineStandardDeviationCalculator(String prefix, DoubleProvider variable, YoRegistry registry)
   {
      this.variable = variable;

      variance = new YoDouble(prefix + "_Variance", registry);
      populationVariance = new YoDouble(prefix + "_PopulationVariance", registry);
      standardDeviation = new YoDouble(prefix + "_StandardDeviation", registry);
      mean = new YoDouble(prefix + "_Mean", registry);
      sumOfSquare = new YoDouble(prefix + "_SumOfSquare", registry);
      numberOfSamples = new YoInteger(prefix + "_NumberOfSamples", registry);

   }

   public void reset()
   {
      standardDeviation.set(0.0);
      variance.set(0.0);
      mean.set(0.0);
      sumOfSquare.set(0.0);
      numberOfSamples.set(0);
   }

   public void update()
   {
      if (variable == null)
         throw new RuntimeException("Variable was never set.");

      update(variable.getValue());
   }

   public void update(double variableValue)
   {
      double totalValue = numberOfSamples.getIntegerValue() * mean.getDoubleValue();
      totalValue += variableValue;

      double previousMean = mean.getDoubleValue();

      numberOfSamples.increment();
      mean.set(totalValue / numberOfSamples.getIntegerValue());

      if (numberOfSamples.getIntegerValue() == 1)
      {
         sumOfSquare.add(MathTools.square(variableValue - mean.getDoubleValue()));
         populationVariance.set(sumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue()));
         variance.set(populationVariance.getDoubleValue());
      }
      else
      {
         sumOfSquare.add((variableValue - previousMean) * (variableValue - mean.getDoubleValue()));
         populationVariance.set(sumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue() - 1));
         variance.set(sumOfSquare.getDoubleValue() / numberOfSamples.getIntegerValue());
      }

      if (numberOfSamples.getValue() < 2)
         standardDeviation.set(0.0);
      else
         standardDeviation.set(Math.sqrt(variance.getDoubleValue()));
   }

   public int getNumberOfSamples()
   {
      return numberOfSamples.getIntegerValue();
   }

   public double getStandardDeviation()
   {
      return standardDeviation.getDoubleValue();
   }

   public double getVariance()
   {
      return variance.getDoubleValue();
   }

   public double getMean()
   {
      return mean.getDoubleValue();
   }
}
