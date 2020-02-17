package us.ihmc.robotics.functionApproximation;

import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLine2D;

public class OnlineLine2DLinearRegression
{
   private final OnlineLeastSquaresRegression onlineLeastSquaresRegression;
   private final YoFrameLine2D line;

   public OnlineLine2DLinearRegression(String prefix, YoVariableRegistry registry)
   {
      onlineLeastSquaresRegression = new OnlineLeastSquaresRegression(prefix, registry);
      line = new YoFrameLine2D(prefix + "_MeanLine", ReferenceFrame.getWorldFrame(), registry);
   }

   private final Point2D firstPointOnLine = new Point2D();
   private final Point2D secondPointOnLine = new Point2D();

   public void update(double x, double y)
   {
      onlineLeastSquaresRegression.update(x, y);
      firstPointOnLine.set(onlineLeastSquaresRegression.getXMean(), onlineLeastSquaresRegression.getYMean());
      double offsetX = onlineLeastSquaresRegression.getXMean() + 0.5; // any offset will do
      secondPointOnLine.set(offsetX, onlineLeastSquaresRegression.computeY(offsetX));
   }

   public Line2DReadOnly getMeanLine()
   {
      return line;
   }

   public double getVariance()
   {
      return onlineLeastSquaresRegression.getVariance();
   }
}
