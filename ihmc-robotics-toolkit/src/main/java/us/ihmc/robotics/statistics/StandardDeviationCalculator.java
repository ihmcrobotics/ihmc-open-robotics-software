package us.ihmc.robotics.statistics;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the Welford's online algorithm
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class StandardDeviationCalculator
{
   private final YoDouble variance;
   private final YoDouble populationVariance;
   private final YoDouble standardDeviation;
   private final YoDouble mean;
   private final YoDouble sumOfSquare;
   private final YoInteger numberOfSamples;

   private final DoubleProvider variable;

   public StandardDeviationCalculator(String prefix, DoubleProvider variable, YoVariableRegistry registry)
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
      double totalValue = numberOfSamples.getIntegerValue() * mean.getDoubleValue();
      totalValue += variable.getValue();

      double previousMean = mean.getDoubleValue();

      numberOfSamples.increment();
      mean.set(totalValue / numberOfSamples.getIntegerValue());

      if (numberOfSamples.getIntegerValue() == 1)
      {
         sumOfSquare.add((variable.getValue() - mean.getDoubleValue()) * (variable.getValue() - mean.getDoubleValue()));
         populationVariance.set(sumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue()));
         variance.set(populationVariance.getDoubleValue());
      }
      else
      {
         sumOfSquare.add((variable.getValue() - previousMean) * (variable.getValue() - mean.getDoubleValue()));
         populationVariance.set(sumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue() - 1));
         variance.set(sumOfSquare.getDoubleValue() / numberOfSamples.getIntegerValue());
      }

      if (MathTools.epsilonEquals(variance.getDoubleValue(), 0.0, 1e-10))
         standardDeviation.set(0.0);
      else
         standardDeviation.set(Math.sqrt(variance.getDoubleValue()));
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
