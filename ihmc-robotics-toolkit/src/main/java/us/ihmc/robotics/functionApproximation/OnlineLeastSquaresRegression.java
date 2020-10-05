package us.ihmc.robotics.functionApproximation;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.statistics.OnlineCovarianceCalculator;
import us.ihmc.robotics.statistics.OnlineStandardDeviationCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Uses the observation that the slope and intercept can be calculated from the x-y correlation, standard deviations, and means.
 * https://machinelearningmastery.com/simple-linear-regression-tutorial-for-machine-learning/
 */
public class OnlineLeastSquaresRegression
{
   private final YoDouble intercept;
   private final YoDouble slope;
   private final YoInteger pointsInRegression;

   private final OnlineCovarianceCalculator covarianceCalculator;

   public OnlineLeastSquaresRegression(String prefix, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(prefix + getClass().getSimpleName());

      intercept = new YoDouble(prefix + "_Intercept", registry);
      slope = new YoDouble(prefix + "_Slope", registry);
      covarianceCalculator = new OnlineCovarianceCalculator(prefix, registry);
      pointsInRegression = new YoInteger(prefix + "PointsInRegression", registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      intercept.set(0.0);
      slope.set(0.0);

      covarianceCalculator.reset();
      pointsInRegression.set(0);
   }

   public double computeY(double x)
   {
      return intercept.getDoubleValue() + slope.getDoubleValue() * x;
   }

   public void update(Point2DReadOnly point)
   {
      update(point.getX(), point.getY());
   }

   public void update(double x, double y)
   {
      covarianceCalculator.update(x, y);
      pointsInRegression.increment();

      if (pointsInRegression.getValue() < 2 || covarianceCalculator.getXStandardDeviation() == 0.0)
         return;

      slope.set(covarianceCalculator.getCorrelation() * covarianceCalculator.getYStandardDeviation() / covarianceCalculator.getXStandardDeviation());
      intercept.set(covarianceCalculator.getYMean() - slope.getDoubleValue() * covarianceCalculator.getXMean());
   }

   public int getPointsInRegression()
   {
      return pointsInRegression.getIntegerValue();
   }

   public double getRSquared()
   {
      return MathTools.square(covarianceCalculator.getCorrelation());
   }

   public double getXStandardDeviation()
   {
      return covarianceCalculator.getXStandardDeviation();
   }

   public double getYStandardDeviation()
   {
      return covarianceCalculator.getYStandardDeviation();
   }

   public double getXMean()
   {
      return covarianceCalculator.getXMean();
   }

   public double getYMean()
   {
      return covarianceCalculator.getYMean();
   }
}
