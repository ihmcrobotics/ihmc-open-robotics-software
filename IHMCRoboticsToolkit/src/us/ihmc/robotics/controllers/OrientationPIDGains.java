package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;

public class OrientationPIDGains implements PID3DGains
{
   private double[] orientationProportionalGains = new double[3];
   private double[] orientationDerivativeGains = new double[3];
   private double[] orientationIntegralGains = new double[3];

   private double orientationMaxIntegralError = 0.0;
   private double orientationMaxDerivativeError = Double.POSITIVE_INFINITY;
   private double orientationMaxProportionalError = Double.POSITIVE_INFINITY;

   private double orientationMaximumFeedback = Double.POSITIVE_INFINITY;
   private double orientationMaximumFeedbackRate = Double.POSITIVE_INFINITY;

   @Override
   public void set(PID3DGainsReadOnly gains)
   {
      for (int i = 0; i < 3; i++)
      {
         orientationProportionalGains[i] = gains.getProportionalGains()[i];
         orientationDerivativeGains[i] = gains.getDerivativeGains()[i];
         orientationIntegralGains[i] = gains.getIntegralGains()[i];
      }

      orientationMaxIntegralError = gains.getMaximumIntegralError();
      orientationMaxDerivativeError = gains.getMaximumDerivativeError();
      orientationMaxProportionalError = gains.getMaximumProportionalError();

      orientationMaximumFeedback = gains.getMaximumFeedback();
      orientationMaximumFeedbackRate = gains.getMaximumFeedbackRate();
   }

   public void setGains(double proportionalGain, double derivativeGain)
   {
      setGains(proportionalGain, derivativeGain, 0.0, 0.0);
   }

   public void setGains(double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError)
   {
      setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
      setIntegralGains(integralGain, integralGain, integralGain, maxIntegralError);
   }

   @Override
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      this.orientationProportionalGains[0] = proportionalGainX;
      this.orientationProportionalGains[1] = proportionalGainY;
      this.orientationProportionalGains[2] = proportionalGainZ;
   }

   @Override
   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.orientationDerivativeGains[0] = derivativeGainX;
      this.orientationDerivativeGains[1] = derivativeGainY;
      this.orientationDerivativeGains[2] = derivativeGainZ;
   }

   @Override
   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      this.orientationIntegralGains[0] = integralGainX;
      this.orientationIntegralGains[1] = integralGainY;
      this.orientationIntegralGains[2] = integralGainZ;
      this.orientationMaxIntegralError = maxIntegralError;
   }

   @Override
   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      orientationMaximumFeedback = maxFeedback;
      orientationMaximumFeedbackRate = maxFeedbackRate;
   }

   @Override
   public void setMaxDerivativeError(double maxDerivativeError)
   {
      orientationMaxDerivativeError = maxDerivativeError;
   }

   @Override
   public void setMaxProportionalError(double maxProportionalError)
   {
      orientationMaxProportionalError = maxProportionalError;
   }

   @Override
   public double[] getProportionalGains()
   {
      return orientationProportionalGains;
   }

   @Override
   public double[] getDerivativeGains()
   {
      return orientationDerivativeGains;
   }

   @Override
   public double[] getIntegralGains()
   {
      return orientationIntegralGains;
   }

   @Override
   public double getMaximumIntegralError()
   {
      return orientationMaxIntegralError;
   }

   @Override
   public double getMaximumFeedback()
   {
      return orientationMaximumFeedback;
   }

   @Override
   public double getMaximumFeedbackRate()
   {
      return orientationMaximumFeedbackRate;
   }

   @Override
   public double getMaximumDerivativeError()
   {
      return orientationMaxDerivativeError;
   }

   @Override
   public double getMaximumProportionalError()
   {
      return orientationMaxProportionalError;
   }
}
