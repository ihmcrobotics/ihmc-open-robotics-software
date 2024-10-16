package us.ihmc.wholeBodyControlCore.pidGains.implementations;

import java.util.Arrays;

import us.ihmc.wholeBodyControlCore.pidGains.GainCalculator;
import us.ihmc.wholeBodyControlCore.pidGains.GainCoupling;
import us.ihmc.wholeBodyControlCore.pidGains.PID3DGainsBasics;
import us.ihmc.wholeBodyControlCore.pidGains.PID3DGainsReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

/**
 * Provides a default implementation for PID gains in three dimensions.
 * <p>
 * If this object is created a {@link GainCoupling} can be specified. This gain coupling is used in
 * case these PID gains will be used to create {@link PID3DGainsBasics}. In that case is it used to
 * determine what YoVariables to create for tuning. Note, that regardless of the specified gain
 * coupling the getters and setters in this implementation are designed for three dimensions.
 * </p>
 */
public class PID3DGains implements PID3DGainsBasics
{
   private final double[] proportionalGains = new double[3];
   private final double[] derivativeGains = new double[3];
   private final double[] integralGains = new double[3];
   private final double[] dampingRatios = new double[3];

   private double maxIntegralError = 0.0;
   private double maxDerivativeError = Double.POSITIVE_INFINITY;
   private double maxProportionalError = Double.POSITIVE_INFINITY;
   private double maxFeedback = Double.POSITIVE_INFINITY;
   private double maxFeedbackRate = Double.POSITIVE_INFINITY;

   private final DoubleProvider maxFeedbackProvider = () -> maxFeedback;
   private final DoubleProvider maxFeedbackRateProvider = () -> maxFeedbackRate;

   public PID3DGains()
   {
   }

   public PID3DGains(PID3DGainsReadOnly other)
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
      updateDerivativeGains();
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      derivativeGains[0] = derivativeGainX;
      derivativeGains[1] = derivativeGainY;
      derivativeGains[2] = derivativeGainZ;
      updateDampingRatios();
   }

   public void setDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      dampingRatios[0] = dampingRatioX;
      dampingRatios[1] = dampingRatioY;
      dampingRatios[2] = dampingRatioZ;
      updateDerivativeGains();
   }

   public void setDampingRatios(double dampingRatio)
   {
      setDampingRatios(dampingRatio, dampingRatio, dampingRatio);
   }

   private void updateDerivativeGains()
   {
      for (int i = 0; i < 3; i++)
      {
         derivativeGains[i] = GainCalculator.computeDerivativeGain(proportionalGains[i], dampingRatios[i]);
      }
   }

   private void updateDampingRatios()
   {
      for (int i = 0; i < 3; i++)
      {
         dampingRatios[i] = GainCalculator.computeDampingRatio(proportionalGains[i], derivativeGains[i]);
      }
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
      if (object instanceof PID3DGains other)
      {
         if (!Arrays.equals(dampingRatios, other.dampingRatios))
            return false;
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
