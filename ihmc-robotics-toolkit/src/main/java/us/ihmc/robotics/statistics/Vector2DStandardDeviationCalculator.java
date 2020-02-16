package us.ihmc.robotics.statistics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the Welford's online algorithm
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class Vector2DStandardDeviationCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble variance;
   private final YoDouble populationVariance;
   private final YoDouble standardDeviation;
   private final YoFrameVector2D mean;
   private final YoDouble sumOfSquare;
   private final YoInteger numberOfSamples;

   private final Vector2DReadOnly variable;

   private final Vector2D previousMean = new Vector2D();
   private final Vector2D totalValue = new Vector2D();

   public Vector2DStandardDeviationCalculator(String prefix, Vector2DReadOnly variable, YoVariableRegistry registry)
   {
      this.variable = variable;

      variance = new YoDouble(prefix + "_Variance", registry);
      populationVariance = new YoDouble(prefix + "_PopulationVariance", registry);
      standardDeviation = new YoDouble(prefix + "_StandardDeviation", registry);
      mean = new YoFrameVector2D(prefix + "_Mean", worldFrame, registry);
      sumOfSquare = new YoDouble(prefix + "_SumOfSquare", registry);
      numberOfSamples = new YoInteger(prefix + "_NumberOfSamples", registry);
   }

   public void reset()
   {
      standardDeviation.set(0.0);
      variance.set(0.0);
      mean.setToZero();
      sumOfSquare.set(0.0);
      numberOfSamples.set(0);
   }

   public void update()
   {
      totalValue.set(mean);
      totalValue.scale(numberOfSamples.getIntegerValue());
      totalValue.add(variable);

      previousMean.set(mean);

      numberOfSamples.increment();
      mean.set(totalValue);
      mean.scale(1.0 / numberOfSamples.getIntegerValue());

      if (numberOfSamples.getIntegerValue() == 1)
      {
         // FIXME this is wrong
         sumOfSquare.add(MathTools.square(variable.dot(mean) - mean.length()));
         populationVariance.set(sumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue()));
         variance.set(populationVariance.getDoubleValue());
      }
      else
      {
         sumOfSquare.add((variable.dot(previousMean) - previousMean.length()) * (variable.dot(mean) - mean.length()));
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

   public Vector2DReadOnly getMean()
   {
      return mean;
   }
}
