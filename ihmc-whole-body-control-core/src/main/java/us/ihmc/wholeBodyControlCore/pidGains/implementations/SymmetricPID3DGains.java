package us.ihmc.wholeBodyControlCore.pidGains.implementations;

import us.ihmc.wholeBodyControlCore.pidGains.PID3DGainsReadOnly;
import us.ihmc.wholeBodyControlCore.pidGains.PIDGainsReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SymmetricPID3DGains implements PID3DGainsReadOnly
{
   private PIDGainsReadOnly gains;

   private final double[] tempProportionalGains = new double[3];
   private final double[] tempDerivativeGains = new double[3];
   private final double[] tempIntegralGains = new double[3];

   public void setGains(PIDGainsReadOnly gains)
   {
      this.gains = gains;
   }

   @Override
   public double[] getProportionalGains()
   {
      return fill(tempProportionalGains, gains.getKp());
   }

   @Override
   public double[] getDerivativeGains()
   {
      return fill(tempDerivativeGains, gains.getKd());
   }

   @Override
   public double[] getIntegralGains()
   {
      return fill(tempIntegralGains, gains.getKi());
   }

   @Override
   public double getMaximumIntegralError()
   {
      return gains.getMaxIntegralError();
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getMaximumProportionalError()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getMaximumFeedback()
   {
      return gains.getMaximumFeedback();
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return gains.getMaximumFeedbackRate();
   }

   @Override
   public DoubleProvider getMaximumFeedbackProvider()
   {
      return gains.getMaximumFeedbackProvider();
   }

   @Override
   public DoubleProvider getMaximumFeedbackRateProvider()
   {
      return gains.getMaximumFeedbackRateProvider();
   }

   private static double[] fill(double[] arrayToFill, double value)
   {
      arrayToFill[0] = value;
      arrayToFill[1] = value;
      arrayToFill[2] = value;
      return arrayToFill;
   }

}
