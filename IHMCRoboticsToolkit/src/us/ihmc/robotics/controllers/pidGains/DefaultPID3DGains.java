package us.ihmc.robotics.controllers.pidGains;

public class DefaultPID3DGains implements PID3DGains
{
   private final GainCoupling gainCoupling;
   private final boolean useDampingRatios;

   private double[] proportionalGains = new double[3];
   private double[] derivativeGains = new double[3];
   private double[] integralGains = new double[3];
   private double[] dampingRatios = new double[3];

   private double maxIntegralError = 0.0;
   private double maxDerivativeError = Double.POSITIVE_INFINITY;
   private double maxProportionalError = Double.POSITIVE_INFINITY;
   private double maxFeedback = Double.POSITIVE_INFINITY;
   private double maxFeedbackRate = Double.POSITIVE_INFINITY;

   public DefaultPID3DGains(GainCoupling gainCoupling, boolean useDampingRatios)
   {
      this.gainCoupling = gainCoupling;
      this.useDampingRatios = useDampingRatios;
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
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      proportionalGains[0] = proportionalGainX;
      proportionalGains[1] = proportionalGainY;
      proportionalGains[2] = proportionalGainZ;

      if (useDampingRatios)
      {
         updateDerivativeGains();
      }
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      if (useDampingRatios)
      {
         throw new RuntimeException("Using damping ratios. Do not set derivative gains directly.");
      }

      derivativeGains[0] = derivativeGainX;
      derivativeGains[1] = derivativeGainY;
      derivativeGains[2] = derivativeGainZ;
   }

   public void setDampingRatios(double dampingRatioX, double dampingRatioY, double dampingRatioZ)
   {
      dampingRatios[0] = dampingRatioX;
      dampingRatios[1] = dampingRatioY;
      dampingRatios[2] = dampingRatioZ;

      if (useDampingRatios)
      {
         updateDerivativeGains();
      }
   }

   private void updateDerivativeGains()
   {
      for (int i = 0; i < 3; i++)
      {
         derivativeGains[i] = GainCalculator.computeDerivativeGain(proportionalGains[i], dampingRatios[i]);
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
   public GainCoupling getGainCoupling()
   {
      return gainCoupling;
   }
}
