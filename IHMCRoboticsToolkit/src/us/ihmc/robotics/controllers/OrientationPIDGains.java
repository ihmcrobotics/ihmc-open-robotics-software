package us.ihmc.robotics.controllers;

public class OrientationPIDGains implements OrientationPIDGainsInterface
{
   private double[] orientationProportionalGains = new double[3];
   private double[] orientationDerivativeGains = new double[3];
   private double[] orientationIntegralGains = new double[3];
   private double orientationMaxIntegralError = 0.0;

   private double orientationMaximumAcceleration = Double.POSITIVE_INFINITY;
   private double orientationMaximumJerk = Double.POSITIVE_INFINITY;


   @Override
   public void set(OrientationPIDGainsInterface gains)
   {
      for (int i = 0; i < 3; i++)
      {
         orientationProportionalGains[i] = gains.getProportionalGains()[i];
         orientationDerivativeGains[i] = gains.getDerivativeGains()[i];
         orientationIntegralGains[i] = gains.getIntegralGains()[i];
      }

      orientationMaxIntegralError = gains.getMaximumIntegralError();
      orientationMaximumAcceleration = gains.getMaximumAcceleration();
      orientationMaximumJerk = gains.getMaximumJerk();
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

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      this.orientationProportionalGains[0] = proportionalGainX;
      this.orientationProportionalGains[1] = proportionalGainY;
      this.orientationProportionalGains[2] = proportionalGainZ;
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.orientationDerivativeGains[0] = derivativeGainX;
      this.orientationDerivativeGains[1] = derivativeGainY;
      this.orientationDerivativeGains[2] = derivativeGainZ;
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      this.orientationIntegralGains[0] = integralGainX;
      this.orientationIntegralGains[1] = integralGainY;
      this.orientationIntegralGains[2] = integralGainZ;
      this.orientationMaxIntegralError = maxIntegralError;
   }

   public void setMaximumAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      orientationMaximumAcceleration = maxAcceleration;
      orientationMaximumJerk = maxJerk;
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
   public double getMaximumAcceleration()
   {
      return orientationMaximumAcceleration;
   }

   @Override
   public double getMaximumJerk()
   {
      return orientationMaximumJerk;
   }
}
