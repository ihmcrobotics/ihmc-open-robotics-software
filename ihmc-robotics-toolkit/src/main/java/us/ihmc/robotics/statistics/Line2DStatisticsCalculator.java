package us.ihmc.robotics.statistics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.functionApproximation.ProbabilityDensityFunction;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the Welford's online algorithm
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class Line2DStatisticsCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble heading;
   private final YoFrameLine2D meanLine;

   private final Vector2D direction = new Vector2D();

   private final Line2DReadOnly lineVariable;

   private final YoDouble positionVariance;
   private final YoDouble positionPopulationVariance;
   private final YoDouble positionStandardDeviation;
   private final YoDouble positionErrorSumOfSquare;

   private final YoDouble directionVariance;
   private final YoDouble directionPopulationVariance;
   private final YoDouble directionStandardDeviation;
   private final YoDouble headingMean;
   private final YoDouble directionErrorSumOfSquare;
   private final YoInteger numberOfSamples;

   private final Point2D previousPositionMean = new Point2D();

   public Line2DStatisticsCalculator(String prefix, Line2DReadOnly lineVariable, YoRegistry registry)
   {
      this.lineVariable = lineVariable;

      heading = new YoDouble(prefix + "_Heading", registry);

      positionVariance = new YoDouble(prefix + "_PositionVariance", registry);
      positionPopulationVariance = new YoDouble(prefix + "_PositionPopulationVariance", registry);
      positionStandardDeviation = new YoDouble(prefix + "_PositionStandardDeviation", registry);
      positionErrorSumOfSquare = new YoDouble(prefix + "_PositionErrorSumOfSquare", registry);

      directionVariance = new YoDouble(prefix + "_DirectionVariance", registry);
      directionPopulationVariance = new YoDouble(prefix + "_DirectionPopulationVariance", registry);
      directionStandardDeviation = new YoDouble(prefix + "_DirectionStandardDeviation", registry);
      headingMean = new YoDouble(prefix + "_HeadingMean", registry);
      meanLine = new YoFrameLine2D(prefix + "_MeanLine", worldFrame, registry);
      directionErrorSumOfSquare = new YoDouble(prefix + "_DirectionErrorSumOfSquare", registry);
      numberOfSamples = new YoInteger(prefix + "_NumberOfLineSamples", registry);
   }

   public void reset()
   {
      positionVariance.set(0.0);
      positionPopulationVariance.set(0.0);
      positionStandardDeviation.set(0.0);
      positionErrorSumOfSquare.set(0.0);

      directionVariance.set(0.0);
      directionPopulationVariance.set(0.0);
      directionStandardDeviation.set(0.0);

      headingMean.set(0.0);
      directionErrorSumOfSquare.set(0.0);
      numberOfSamples.set(0);
   }

   public void update()
   {
      Point2DReadOnly incomingPosition = lineVariable.getPoint();

      heading.set(calculateHeading(lineVariable.getDirection()));
      direction.set(Math.cos(heading.getDoubleValue()), Math.sin(heading.getDoubleValue()));

      double totalHeadingValue = numberOfSamples.getIntegerValue() * headingMean.getDoubleValue() + heading.getDoubleValue();
      double previousHeadingMean = headingMean.getDoubleValue();

      numberOfSamples.increment();

      headingMean.set(totalHeadingValue / numberOfSamples.getIntegerValue());
      meanLine.getDirection().set(Math.cos(headingMean.getDoubleValue()), Math.sin(headingMean.getDoubleValue()));

      meanLine.orthogonalProjection(incomingPosition, previousPositionMean);
      meanLine.getPoint().interpolate(previousPositionMean, incomingPosition, 1.0 / numberOfSamples.getIntegerValue());

      if (numberOfSamples.getIntegerValue() > 1)
      {
         double difference = AngleTools.computeAngleDifferenceMinusPiToPi(heading.getDoubleValue(), headingMean.getDoubleValue());
         double previousDifference = AngleTools.computeAngleDifferenceMinusPiToPi(heading.getDoubleValue(), previousHeadingMean);
         directionErrorSumOfSquare.add(difference * previousDifference);
         directionPopulationVariance.set(directionErrorSumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue() - 1));
         directionVariance.set(directionErrorSumOfSquare.getDoubleValue() / numberOfSamples.getIntegerValue());

         positionErrorSumOfSquare.add(incomingPosition.distance(previousPositionMean) * incomingPosition.distance(meanLine.getPoint()));
         positionPopulationVariance.set(positionErrorSumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue() - 1));
         positionVariance.set(positionErrorSumOfSquare.getDoubleValue() / numberOfSamples.getIntegerValue());
      }

      if (MathTools.epsilonEquals(positionVariance.getDoubleValue(), 0.0, 1e-10))
         positionStandardDeviation.set(0.0);
      else
         positionStandardDeviation.set(Math.sqrt(positionVariance.getDoubleValue()));

      if (MathTools.epsilonEquals(directionVariance.getDoubleValue(), 0.0, 1e-10))
         directionStandardDeviation.set(0.0);
      else
         directionStandardDeviation.set(Math.sqrt(directionVariance.getDoubleValue()));
   }

   public double getPositionStandardDeviation()
   {
      return positionStandardDeviation.getDoubleValue();
   }

   public double getPositionVariance()
   {
      return positionVariance.getValue();
   }

   public double getDirectionStandardDeviation()
   {
      return directionStandardDeviation.getDoubleValue();
   }

   public double getDirectionVariance()
   {
      return directionVariance.getDoubleValue();
   }

   public Line2DReadOnly getLineMean()
   {
      return meanLine;
   }

   static double calculateHeading(Vector2DReadOnly direction)
   {
      return calculateHeading(direction.getX(), direction.getY());
   }

   private static double calculateHeading(double deltaX, double deltaY)
   {
      double angle = Math.atan2(deltaY, deltaX);
      if (angle < 0.0)
         angle += Math.PI;
      return angle;
   }

   public double getProbabilityPointIsOnLine(Point2DReadOnly point)
   {
      double distanceToMean = getLineMean().distance(point);
      return ProbabilityDensityFunction.getProbabilityUsingNormalDistribution(distanceToMean, 0.0, getPositionStandardDeviation());
   }
}
