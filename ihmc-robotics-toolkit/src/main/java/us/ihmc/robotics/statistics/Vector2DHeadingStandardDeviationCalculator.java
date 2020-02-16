package us.ihmc.robotics.statistics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the Welford's online algorithm
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class Vector2DHeadingStandardDeviationCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble heading;

   private final Vector2DReadOnly variable;

   private final YoDouble variance;
   private final YoDouble populationVariance;
   private final YoDouble standardDeviation;
   private final YoDouble mean;
   private final YoDouble sumOfSquare;
   private final YoInteger numberOfSamples;

   public Vector2DHeadingStandardDeviationCalculator(String prefix, Vector2DReadOnly variable, YoVariableRegistry registry)
   {
      this.variable = variable;

      heading = new YoDouble(prefix + "_Heading", registry);

      variance = new YoDouble(prefix + "_Variance", registry);
      populationVariance = new YoDouble(prefix + "_HeadingPopulationVariance", registry);
      standardDeviation = new YoDouble(prefix + "_HeadingStandardDeviation", registry);
      mean = new YoDouble(prefix + "_HeadingMean", registry);
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
      heading.set(calculateHeading(variable));

      double totalValue = numberOfSamples.getIntegerValue() * mean.getDoubleValue() + heading.getDoubleValue();

      double previousMean = mean.getDoubleValue();

      numberOfSamples.increment();
      mean.set(totalValue / numberOfSamples.getIntegerValue());

      if (numberOfSamples.getIntegerValue() > 1)
      {
         double difference = AngleTools.computeAngleDifferenceMinusPiToPi(heading.getDoubleValue(), mean.getDoubleValue());
         double previousDifference = AngleTools.computeAngleDifferenceMinusPiToPi(heading.getDoubleValue(), previousMean);
         sumOfSquare.add(difference * previousDifference);
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

   private static double calculateHeading(Vector2DReadOnly direction)
   {
      return calculateHeading(direction.getX(), direction.getY());
   }

   private static double calculateHeading(double deltaX, double deltaY)
   {
      return Math.atan2(deltaY, deltaX);
   }
}
