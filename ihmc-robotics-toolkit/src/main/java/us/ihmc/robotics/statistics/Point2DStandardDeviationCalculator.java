package us.ihmc.robotics.statistics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the Welford's online algorithm
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class Point2DStandardDeviationCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble variance;
   private final YoDouble populationVariance;
   private final YoDouble standardDeviation;
   private final YoFramePoint2D mean;
   private final YoDouble sumOfSquare;
   private final YoInteger numberOfSamples;

   private final Point2DReadOnly variable;

   private final Point2D previousMean = new Point2D();

   public Point2DStandardDeviationCalculator(String prefix, Point2DReadOnly variable, YoRegistry registry)
   {
      this.variable = variable;

      variance = new YoDouble(prefix + "_Variance", registry);
      populationVariance = new YoDouble(prefix + "_PopulationVariance", registry);
      standardDeviation = new YoDouble(prefix + "_StandardDeviation", registry);
      mean = new YoFramePoint2D(prefix + "_Mean", worldFrame, registry);
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
      previousMean.set(mean);

      numberOfSamples.increment();
      mean.interpolate(previousMean, variable, 1.0 / numberOfSamples.getIntegerValue());

      if (numberOfSamples.getIntegerValue() == 1)
      {
         sumOfSquare.add(variable.distanceSquared(mean));
         populationVariance.set(sumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue()));
         variance.set(populationVariance.getDoubleValue());
      }
      else
      {
         sumOfSquare.add(variable.distance(previousMean) * variable.distance(mean));
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

   public Point2DReadOnly getMean()
   {
      return mean;
   }
}
