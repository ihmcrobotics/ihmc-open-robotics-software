package us.ihmc.robotics.statistics;

import us.ihmc.robotics.MathTools;

/**
 *
 * @author Nicolas Gerig
 * One pass efficient way of calculating variances and standard deviation efficiently
 * Based On:
 * http://www.cs.berkeley.edu/~mhoemmen/cs194/Tutorials/variance.pdf
 *
 */
public class OnePassMeanAndStandardDeviation
{
   private int k = 0;
   private double sum;

   private double mK;
   private double qK;

   /**
    * Updates information needed for average and variances for one-pass
    * @param value
    */
   public void compute(double value)
   {
      if (k == 0)
         computeFirst(value);
      else
         computeOther(value);
   }

   private void computeFirst(double value)
   {
      k = 1;
      sum = value;
      qK = 0.0;
      mK = value;
   }

   private void computeOther(double value)
   {
      k++;
      sum = sum + value;
      
      double mKLast = mK;
      double kDouble = (double)k;
      mK = mKLast + (value - mKLast) / kDouble;
      qK = qK + (kDouble - 1.0) * MathTools.square(value - mKLast) / kDouble;
   }
   
   /**
    * @return n, the number of measurements used for average and standard deviation so far
    */
   public int getNumberOfTotalMeasurements()
   {
      int n = k;
      return n;
   }
   
   /**
    * @return mu, the average of all measurements so far
    */
   public double getAverage()
   {
      if (k == 0)
         throw new InsufficientMeasurementsException("Average can not be defined, needs at least 1 measurment.");
      
      double mu = sum / (double)k;
      return mu;
   }
   
   /**
    * @return sigmaSquare, the (population) variance, based on all measurements so far
    */
   public double getVariance()
   {
      if (k == 0)
         throw new InsufficientMeasurementsException("Variance can not be defined, needs at least 1 measurment.");
      
      double sigmaSquare = qK / (double)k;
      return sigmaSquare;
   }

   /**
    * @return sigma, the standard deviation, based on all measurements so far
    */
   public double getStandardDeviation()
   {
      double sigma = Math.sqrt(getVariance());
      return sigma;
   }
   
   /**
    * @return sSquare, the sample variance, based on all measurements so far
    */
   public double getSampleVariance()
   {
      if (k <= 1)
         throw new InsufficientMeasurementsException("SampleVariance can not be defined, needs at least 2 measurments.");
      
      double sSquare = qK / (double)(k-1);
      return sSquare;
   }
   
   class InsufficientMeasurementsException extends RuntimeException
   {
      public InsufficientMeasurementsException(String string)
      {
         super(string);
      }

      private static final long serialVersionUID = 6173538739891490316L;
   }
   
}
