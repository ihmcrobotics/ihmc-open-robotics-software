package us.ihmc.robotics.functionApproximation;

import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.statistics.OnlineStandardDeviationCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameLine2D;

public class OnlineLine2DLinearRegression
{
   private final OnlineLeastSquaresRegression onlineLeastSquaresRegression;
   private final YoFrameLine2D line;

   private final YoDouble transverseResidual;
   private final YoDouble inlineResidual;

   private final OnlineStandardDeviationCalculator transverseDeviationCalculator;
   private final OnlineStandardDeviationCalculator inlineDeviationCalculator;

   public OnlineLine2DLinearRegression(String prefix, YoVariableRegistry registry)
   {
      onlineLeastSquaresRegression = new OnlineLeastSquaresRegression(prefix, registry);
      transverseDeviationCalculator = new OnlineStandardDeviationCalculator(prefix + "Transverse", registry);
      inlineDeviationCalculator = new OnlineStandardDeviationCalculator(prefix + "Inline", registry);

      line = new YoFrameLine2D(prefix + "_MeanLine", ReferenceFrame.getWorldFrame(), registry);
      transverseResidual = new YoDouble(prefix + "_TransverseResidual", registry);
      inlineResidual = new YoDouble(prefix + "_InlineResidual", registry);
   }

   private final Point2D firstPointOnLine = new Point2D();
   private final Point2D secondPointOnLine = new Point2D();

   public void reset()
   {
      onlineLeastSquaresRegression.reset();
      transverseDeviationCalculator.reset();
      inlineDeviationCalculator.reset();
      inlineResidual.set(0.0);
      transverseResidual.set(0.0);
   }

   public void update(Point2DReadOnly point)
   {
      update(point.getX(), point.getY());
   }

   public void update(double x, double y)
   {
      onlineLeastSquaresRegression.update(x, y);
      firstPointOnLine.set(onlineLeastSquaresRegression.getXMean(), onlineLeastSquaresRegression.getYMean());
      double offsetX = onlineLeastSquaresRegression.getXMean() + 0.5; // any offset will do
      secondPointOnLine.set(offsetX, onlineLeastSquaresRegression.computeY(offsetX));

      line.set(firstPointOnLine, secondPointOnLine);
      transverseResidual.set(EuclidGeometryTools.distanceFromPoint2DToLine2D(x, y, line.getPoint(), line.getDirection()));
      if (!transverseResidual.isNaN())
         transverseDeviationCalculator.update(transverseResidual.getDoubleValue());

      inlineResidual.set(dotProduct(firstPointOnLine, x, y, line.getDirection()));
      if (!inlineResidual.isNaN())
         inlineDeviationCalculator.update(inlineResidual.getDoubleValue());
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

   public double getTransverseVariance()
   {
      return transverseDeviationCalculator.getVariance();
   }

   public double getTransverseStandardDeviation()
   {
      return transverseDeviationCalculator.getStandardDeviation();
   }

   public double getInlineVariance()
   {
      return inlineDeviationCalculator.getVariance();
   }

   public double getProbabilityPointIsOnLine(Point2DReadOnly point)
   {
      double distanceToMean = line.distance(point);
      return ProbabilityDensityFunction.getProbabilityUsingNormalDistribution(distanceToMean, 0.0, getTransverseStandardDeviation());
   }
}
