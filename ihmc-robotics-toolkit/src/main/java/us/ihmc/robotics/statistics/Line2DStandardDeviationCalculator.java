package us.ihmc.robotics.statistics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the Welford's online algorithm
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class Line2DStandardDeviationCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble heading;
   private final YoFrameLine2D meanLine;

   private final Vector2D direction = new Vector2D();

   private final Line2DReadOnly lineVariable;

   private final YoDouble directionVariance;
   private final YoDouble directionPopulationVariance;
   private final YoDouble directionStandardDeviation;
   private final YoDouble headingMean;
   private final YoDouble directionErrorSumOfSquare;
   private final YoInteger numberOfSamples;

   // FIXME don't use the point statistics, since a point anywhere on a line defines that point
   private final Point2DStandardDeviationCalculator pointStatistics;

   public Line2DStandardDeviationCalculator(String prefix, Line2DReadOnly lineVariable, YoVariableRegistry registry)
   {
      this.lineVariable = lineVariable;
      pointStatistics = new Point2DStandardDeviationCalculator(prefix, lineVariable.getPoint(), registry);

      heading = new YoDouble(prefix + "_Heading", registry);

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
      directionStandardDeviation.set(0.0);
      directionVariance.set(0.0);
      headingMean.set(0.0);
      directionErrorSumOfSquare.set(0.0);
      numberOfSamples.set(0);
   }

   public void update()
   {
      pointStatistics.update();
      meanLine.setPoint(pointStatistics.getMean());

      heading.set(calculateHeading(lineVariable.getDirection()));
      direction.set(Math.cos(heading.getDoubleValue()), Math.sin(heading.getDoubleValue()));

      double totalHeadingValue = numberOfSamples.getIntegerValue() * headingMean.getDoubleValue() + heading.getDoubleValue();
      double previousHeadingMean = headingMean.getDoubleValue();

      numberOfSamples.increment();

      headingMean.set(totalHeadingValue / numberOfSamples.getIntegerValue());
      meanLine.setDirection(Math.cos(headingMean.getDoubleValue()), Math.sin(headingMean.getDoubleValue()));

      if (numberOfSamples.getIntegerValue() > 1)
      {
         double difference = AngleTools.computeAngleDifferenceMinusPiToPi(heading.getDoubleValue(), headingMean.getDoubleValue());
         double previousDifference = AngleTools.computeAngleDifferenceMinusPiToPi(heading.getDoubleValue(), previousHeadingMean);
         directionErrorSumOfSquare.add(difference * previousDifference);
         directionPopulationVariance.set(directionErrorSumOfSquare.getDoubleValue() / (numberOfSamples.getIntegerValue() - 1));
         directionVariance.set(directionErrorSumOfSquare.getDoubleValue() / numberOfSamples.getIntegerValue());
      }

      if (MathTools.epsilonEquals(directionVariance.getDoubleValue(), 0.0, 1e-10))
         directionStandardDeviation.set(0.0);
      else
         directionStandardDeviation.set(Math.sqrt(directionVariance.getDoubleValue()));
   }

   public double getPositionStandardDeviation()
   {
      return pointStatistics.getStandardDeviation();
   }

   public double getPositionVariance()
   {
      return pointStatistics.getVariance();
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
}
