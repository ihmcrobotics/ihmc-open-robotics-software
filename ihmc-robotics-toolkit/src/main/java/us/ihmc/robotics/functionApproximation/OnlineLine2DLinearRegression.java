package us.ihmc.robotics.functionApproximation;

import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.statistics.OnlineStandardDeviationCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameLine2D;

public class OnlineLine2DLinearRegression
{
   private final OnlineLeastSquaresRegression onlineLeastSquaresRegression;
   private final YoFrameLine2D line;

   private final YoDouble residual;

   private final OnlineStandardDeviationCalculator deviationCalculator;

   public OnlineLine2DLinearRegression(String prefix, YoVariableRegistry registry)
   {
      onlineLeastSquaresRegression = new OnlineLeastSquaresRegression(prefix, registry);
      deviationCalculator = new OnlineStandardDeviationCalculator(prefix, registry);

      line = new YoFrameLine2D(prefix + "_MeanLine", ReferenceFrame.getWorldFrame(), registry);
      residual = new YoDouble(prefix + "_NormalResidual", registry);
   }

   private final Point2D firstPointOnLine = new Point2D();
   private final Point2D secondPointOnLine = new Point2D();

   public void reset()
   {
      onlineLeastSquaresRegression.reset();
      deviationCalculator.reset();
      residual.set(0.0);
   }

   public void update(double x, double y)
   {
      onlineLeastSquaresRegression.update(x, y);
      firstPointOnLine.set(onlineLeastSquaresRegression.getXMean(), onlineLeastSquaresRegression.getYMean());
      double offsetX = onlineLeastSquaresRegression.getXMean() + 0.5; // any offset will do
      secondPointOnLine.set(offsetX, onlineLeastSquaresRegression.computeY(offsetX));

      line.set(firstPointOnLine, secondPointOnLine);
      residual.set(EuclidGeometryTools.distanceFromPoint2DToLine2D(x, y, line.getPoint(), line.getDirection()));
      if (!residual.isNaN())
         deviationCalculator.update(residual.getDoubleValue());
   }

   public Line2DReadOnly getMeanLine()
   {
      return line;
   }

   public double getVariance()
   {
      return deviationCalculator.getVariance();
   }

   public double getStandardDeviation()
   {
      return deviationCalculator.getStandardDeviation();
   }

   public double getProbabilityPointIsOnLine(Point2DReadOnly point)
   {
      double distanceToMean = line.distance(point);
      return ProbabilityDensityFunction.getProbabilityUsingNormalDistribution(distanceToMean, 0.0, getStandardDeviation());
   }
}
