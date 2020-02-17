package us.ihmc.robotics.functionApproximation;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/** Uses a combination of the Welford's algorithm for calculating
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
public class WeightedLeastSquaresRegression
{
   private double b0 = 0.0;
   private double b1 = 0.0;
   private int numberOfIterations = 0;
   private double meanX = 0.0;
   private double meanY = 0.0;
   private double coMoment = 0.0;
   private double sampleCovariance = 0.0;
   private double sampleXVariance= 0.0;
   private double sampleYVariance= 0.0;
   private double sampleXStandardDeviation= 0.0;
   private double sampleYStandardDeviation = 0.0;
   private double populationCovariance = 0.0;
   private double populationXVariance= 0.0;
   private double populationYVariance= 0.0;
   private double populationXStandardDeviation = 0.0;
   private double populationYStandardDeviation = 0.0;
   private double sumOfSquaredXError = 0.0;
   private double sumOfSquaredYError = 0.0;
   private double populationCorrelation = 0.0;
   private double sampleCorrelation = 0.0;


   public void reset()
   {
      b0 = 0.0;
      b1 = 0.0;
      numberOfIterations = 0;
      meanX = 0.0;
      meanY = 0.0;
      coMoment = 0.0;
      sumOfSquaredXError = 0.0;
      sumOfSquaredYError = 0.0;
   }

   public double computeY(double x)
   {
      return b0 + b1 * x;
   }

   public void update(Point2DReadOnly point)
   {
      update(point.getX(), point.getY());
   }

   public void update(double x, double y)
   {
      numberOfIterations++;

      double dx = x - meanX;
      double dy = y - meanY;
      meanX += dx / numberOfIterations;
      meanY += dy / numberOfIterations;

      sumOfSquaredXError += (x - meanX) * dx;
      sumOfSquaredYError += (y - meanY) * dy;

      coMoment += dx * (y - meanY);
      populationCovariance = coMoment / numberOfIterations;
      sampleCovariance = coMoment / (numberOfIterations - 1);

      sampleXVariance = sumOfSquaredXError / (numberOfIterations - 1);
      sampleYVariance = sumOfSquaredYError / (numberOfIterations - 1);

      populationXVariance = sumOfSquaredXError / numberOfIterations;
      populationYVariance = sumOfSquaredYError / numberOfIterations;

      populationXStandardDeviation = Math.sqrt(populationXVariance);
      populationYStandardDeviation = Math.sqrt(populationYVariance);

      sampleXStandardDeviation = Math.sqrt(sampleXVariance);
      sampleYStandardDeviation = Math.sqrt(sampleYVariance);

      populationCorrelation = populationCovariance / (populationXStandardDeviation * populationYStandardDeviation);
      sampleCorrelation = sampleCovariance / (sampleXStandardDeviation * sampleYStandardDeviation);

      b1 = populationCorrelation * populationYStandardDeviation / populationXStandardDeviation;
      b0 = meanY - b1 * meanX;
   }
}
