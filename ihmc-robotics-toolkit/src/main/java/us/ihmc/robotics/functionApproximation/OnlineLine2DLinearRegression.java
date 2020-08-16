package us.ihmc.robotics.functionApproximation;

import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class OnlineLine2DLinearRegression
{
   private final OnlineLeastSquaresRegression onlineLeastSquaresRegression;
   private final YoFrameLine2D line;

   private final Vector2D deviationVector = new Vector2D();
   private final YoDouble transverseStandardDeviation;
   private final YoDouble inlineStandardDeviation;

   public OnlineLine2DLinearRegression(String prefix, YoRegistry registry)
   {
      onlineLeastSquaresRegression = new OnlineLeastSquaresRegression(prefix, registry);
      transverseStandardDeviation = new YoDouble(prefix + "TransverseStandardDeviation", registry);
      inlineStandardDeviation = new YoDouble(prefix + "InlineStandardDeviation", registry);

      line = new YoFrameLine2D(prefix + "_MeanLine", ReferenceFrame.getWorldFrame(), registry);
   }

   private final Point2D firstPointOnLine = new Point2D();
   private final Point2D secondPointOnLine = new Point2D();

   public void reset()
   {
      onlineLeastSquaresRegression.reset();
      transverseStandardDeviation.set(0.0);
      inlineStandardDeviation.set(0.0);
   }

   public void update(Point2DReadOnly point)
   {
      update(point.getX(), point.getY());
   }

   public void update(double x, double y)
   {
      onlineLeastSquaresRegression.update(x, y);

      if (onlineLeastSquaresRegression.getPointsInRegression() > 1)
      {
         firstPointOnLine.set(onlineLeastSquaresRegression.getXMean(), onlineLeastSquaresRegression.getYMean());
         double offsetX = onlineLeastSquaresRegression.getXMean() + 0.5; // any offset will do
         secondPointOnLine.set(offsetX, onlineLeastSquaresRegression.computeY(offsetX));

         line.set(firstPointOnLine, secondPointOnLine);

         if (line.getDirection().containsNaN())
         {
            // The points are all at the same location causing the regression to fail.
            return;
         }

         Vector2DReadOnly direction = line.getDirection();
         deviationVector.set(Math.signum(direction.getX()) * onlineLeastSquaresRegression.getXStandardDeviation(), Math.signum(direction.getY()) * onlineLeastSquaresRegression.getYStandardDeviation());

         inlineStandardDeviation.set(Math.abs(deviationVector.dot(direction)));
         transverseStandardDeviation.set(Math.abs(deviationVector.cross(direction)));
      }
   }

   public static double dotProduct(Point2DReadOnly start1, double end1X, double end1Y, Vector2DReadOnly vector2)
   {
      double vector1X = end1X- start1.getX();
      double vector1Y = end1Y - start1.getY();

      return vector1X * vector2.getX() + vector1Y * vector2.getY();
   }

   public Line2DReadOnly getMeanLine()
   {
      return line;
   }

   public double getXStandardDeviation()
   {
      return onlineLeastSquaresRegression.getXStandardDeviation();
   }

   public double getYStandardDeviation()
   {
      return onlineLeastSquaresRegression.getYStandardDeviation();
   }

   public double getTransverseStandardDeviation()
   {
      return transverseStandardDeviation.getDoubleValue();
   }

   public double getInlineStandardDeviation()
   {
      return inlineStandardDeviation.getDoubleValue();
   }

   public double getProbabilityPointIsOnLine(Point2DReadOnly point)
   {
      double distanceToMean = line.distance(point);
      return ProbabilityDensityFunction.getProbabilityUsingNormalDistribution(distanceToMean, 0.0, getTransverseStandardDeviation());
   }
}
