package us.ihmc.robotics.controllers.pidGains;

public interface PID3DGains extends PID3DGainsReadOnly
{
   public abstract void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ);

   public abstract void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ);

   public abstract void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError);

   public abstract void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate);

   public abstract void setMaxDerivativeError(double maxDerivativeError);

   public abstract void setMaxProportionalError(double maxProportionalError);

   public default void setProportionalGains(double[] proportionalGains)
   {
      setProportionalGains(proportionalGains[0], proportionalGains[1], proportionalGains[2]);
   }

   public default void setDerivativeGains(double[] derivativeGains)
   {
      setDerivativeGains(derivativeGains[0], derivativeGains[1], derivativeGains[2]);
   }

   public default void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      setIntegralGains(integralGains[0], integralGains[1], integralGains[2], maxIntegralError);
   }

   public default void set(PID3DGainsReadOnly other)
   {
      setProportionalGains(other.getProportionalGains());
      setDerivativeGains(other.getDerivativeGains());
      setIntegralGains(other.getIntegralGains(), other.getMaximumIntegralError());
      setMaxFeedbackAndFeedbackRate(other.getMaximumFeedback(), other.getMaximumFeedbackRate());
      setMaxDerivativeError(other.getMaximumDerivativeError());
      setMaxProportionalError(other.getMaximumProportionalError());
   }
}
