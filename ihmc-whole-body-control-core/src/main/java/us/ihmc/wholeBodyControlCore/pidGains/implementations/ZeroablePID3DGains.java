package us.ihmc.wholeBodyControlCore.pidGains.implementations;

import java.util.Arrays;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.wholeBodyControlCore.pidGains.PID3DGainsBasics;
import us.ihmc.wholeBodyControlCore.pidGains.PID3DGainsReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

/**
 * Provides a default implementation for PID gains in three dimensions without damping ratios.
 * 
 */
public class ZeroablePID3DGains implements PID3DGainsBasics
{
   private final double[] proportionalGains = new double[3];
   private final double[] derivativeGains = new double[3];
   private final double[] integralGains = new double[3];

   private double maxIntegralError = 0.0;
   private double maxDerivativeError = Double.POSITIVE_INFINITY;
   private double maxProportionalError = Double.POSITIVE_INFINITY;
   private double maxFeedback = Double.POSITIVE_INFINITY;
   private double maxFeedbackRate = Double.POSITIVE_INFINITY;

   private final DoubleProvider maxFeedbackProvider = () -> maxFeedback;
   private final DoubleProvider maxFeedbackRateProvider = () -> maxFeedbackRate;

   public ZeroablePID3DGains()
   {
   }

   public ZeroablePID3DGains(PID3DGainsReadOnly other)
   {
      set(other);
   }

   @Override
   public double[] getProportionalGains()
   {
      return proportionalGains;
   }

   @Override
   public double[] getDerivativeGains()
   {
      return derivativeGains;
   }

   @Override
   public double[] getIntegralGains()
   {
      return integralGains;
   }

   @Override
   public double getMaximumIntegralError()
   {
      return maxIntegralError;
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return maxDerivativeError;
   }

   @Override
   public double getMaximumProportionalError()
   {
      return maxProportionalError;
   }

   @Override
   public double getMaximumFeedback()
   {
      return maxFeedback;
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return maxFeedbackRate;
   }

   @Override
   public DoubleProvider getMaximumFeedbackProvider()
   {
      return maxFeedbackProvider;
   }

   @Override
   public DoubleProvider getMaximumFeedbackRateProvider()
   {
      return maxFeedbackRateProvider;
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalGains[0] = proportionalGainX;
      proportionalGains[1] = proportionalGainY;
      proportionalGains[2] = proportionalGainZ;
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeGains[0] = derivativeGainX;
      derivativeGains[1] = derivativeGainY;
      derivativeGains[2] = derivativeGainZ;
   }


   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      integralGains[0] = integralGainX;
      integralGains[1] = integralGainY;
      integralGains[2] = integralGainZ;
      this.maxIntegralError = maxIntegralError;
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      this.maxFeedback = maxFeedback;
      this.maxFeedbackRate = maxFeedbackRate;
   }

   @Override
   public void setMaxDerivativeError(double maxDerivativeError)
   {
      this.maxDerivativeError = maxDerivativeError;
   }

   @Override
   public void setMaxProportionalError(double maxProportionalError)
   {
      this.maxProportionalError = maxProportionalError;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ZeroablePID3DGains)
      {
         ZeroablePID3DGains other = (ZeroablePID3DGains) object;
         if (!PID3DGainsBasics.super.equals(other))
            return false;
         return true;
      }
      else if (object instanceof PID3DGainsReadOnly)
      {
         return PID3DGainsBasics.super.equals((PID3DGainsReadOnly) object);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": kp: " + Arrays.toString(proportionalGains) + ", kd: " + Arrays.toString(derivativeGains) + ", ki: "
            + Arrays.toString(integralGains);
   }
}
