package us.ihmc.robotics.functionApproximation;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.statistics.OnlineCovarianceCalculator;
import us.ihmc.robotics.statistics.OnlineStandardDeviationCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Uses the observation that the slope and intercept can be calculated from the x-y correlation, standard deviations, and means.
 * https://machinelearningmastery.com/simple-linear-regression-tutorial-for-machine-learning/
 */
public class OnlineLeastSquaresRegression
{
   private final YoDouble intercept;
   private final YoDouble slope;

   private final YoDouble residual;

   private final OnlineCovarianceCalculator covarianceCalculator;
   private final OnlineStandardDeviationCalculator outputStandardDeviationCalculator;


   public OnlineLeastSquaresRegression(String prefix, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      intercept = new YoDouble(prefix + "_Intercept", registry);
      slope = new YoDouble(prefix + "_Slope", registry);
      residual = new YoDouble(prefix + "_Residual", registry);
      covarianceCalculator = new OnlineCovarianceCalculator(prefix, registry);
      outputStandardDeviationCalculator = new OnlineStandardDeviationCalculator(prefix, registry);


      parentRegistry.addChild(registry);
   }


   public void reset()
   {
      intercept.set(0.0);
      slope.set(0.0);
      residual.set(0.0);

      outputStandardDeviationCalculator.reset();
      covarianceCalculator.reset();
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

      slope.set(covarianceCalculator.getCorrelation() * covarianceCalculator.getYStandardDeviation() / covarianceCalculator.getXStandardDeviation());
      intercept.set(covarianceCalculator.getYMean() - slope.getDoubleValue() * covarianceCalculator.getXMean());

      residual.set(computeY(x) - y);
      if (!residual.isNaN())
         outputStandardDeviationCalculator.update(residual.getDoubleValue());
   }

   public double getRSquared()
   {
      return MathTools.square(covarianceCalculator.getCorrelation());
   }

   public double getVariance()
   {
      return outputStandardDeviationCalculator.getVariance();
   }

   public double getStandardDeviation()
   {
      return outputStandardDeviationCalculator.getStandardDeviation();
   }
}
