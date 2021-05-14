package us.ihmc.robotics.statistics;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/** Uses a combination of the Welford's algorithm for calculating
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class OnlineCovarianceCalculator
{
   private final YoDouble coMoment;

   private final YoDouble correlation;
   private final YoDouble covariance;

   private final OnlineStandardDeviationCalculator xCalculator;
   private final OnlineStandardDeviationCalculator yCalculator;

   public OnlineCovarianceCalculator(String prefix, YoRegistry registry)
   {
      coMoment = new YoDouble(prefix + "_CoMoment", registry);
      covariance = new YoDouble(prefix + "_Covariance", registry);
      correlation = new YoDouble(prefix + "_Correlation", registry);
      xCalculator = new OnlineStandardDeviationCalculator(prefix + "X", registry);
      yCalculator = new OnlineStandardDeviationCalculator(prefix + "Y", registry);
   }


   public void reset()
   {
      coMoment.set(0.0);
      correlation.set(0.0);
      covariance.set(0.0);

      xCalculator.reset();
      yCalculator.reset();
   }

   public void update(double x, double y)
   {
      double xError = x - xCalculator.getMean();

      xCalculator.update(x);
      yCalculator.update(y);

      coMoment.add(xError * (y - yCalculator.getMean()));
      covariance.set(coMoment.getDoubleValue() / xCalculator.getNumberOfSamples());
      correlation.set(covariance.getDoubleValue() / (xCalculator.getStandardDeviation() * yCalculator.getStandardDeviation()));
   }

   public double getCorrelation()
   {
      return correlation.getDoubleValue();
   }

   public double getXStandardDeviation()
   {
      return xCalculator.getStandardDeviation();
   }

   public double getYStandardDeviation()
   {
      return yCalculator.getStandardDeviation();
   }

   public double getXMean()
   {
      return xCalculator.getMean();
   }

   public double getYMean()
   {
      return yCalculator.getMean();
   }
}
