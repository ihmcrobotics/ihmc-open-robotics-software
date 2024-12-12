package us.ihmc.robotics.controllers.pidGains.implementations;

import java.util.Arrays;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;

/**
 * Provides a default implementation for PID gains in three dimensions without damping ratios.
 * 
 */
public class ZeroablePID3DGains implements PID3DGains, Settable<ZeroablePID3DGains>
{
   private double[] proportionalGains = new double[3];
   private double[] derivativeGains = new double[3];
   private double[] integralGains = new double[3];
   private double[] dampingRatios = new double[3];

   private double maxIntegralError = 0.0;
   private double maxDerivativeError = Double.POSITIVE_INFINITY;
   private double maxProportionalError = Double.POSITIVE_INFINITY;
   private double maxFeedback = Double.POSITIVE_INFINITY;
   private double maxFeedbackRate = Double.POSITIVE_INFINITY;

   public ZeroablePID3DGains()
   {
   }

   public ZeroablePID3DGains(PID3DGainsReadOnly other)
   {
      set(other);
   }

   @Override
   public void set(ZeroablePID3DGains other)
   {
      PID3DGains.super.set(other);
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
   public double[] getDampingRatios()
   {
      return dampingRatios;
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
   public void setDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      dampingRatios[0] = dampingRatioX;
      dampingRatios[1] = dampingRatioY;
      dampingRatios[2] = dampingRatioZ;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ZeroablePID3DGains)
      {
         ZeroablePID3DGains other = (ZeroablePID3DGains) object;
         if (!PID3DGains.super.equals(other))
            return false;
         return true;
      }
      else if (object instanceof PID3DGainsReadOnly)
      {
         return PID3DGains.super.equals((PID3DGainsReadOnly) object);
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
