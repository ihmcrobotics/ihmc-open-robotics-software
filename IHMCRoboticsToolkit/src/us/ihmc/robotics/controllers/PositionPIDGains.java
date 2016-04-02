package us.ihmc.robotics.controllers;

public class PositionPIDGains implements PositionPIDGainsInterface
{
   private double[] positionProportionalGains = new double[3];
   private double[] positionDerivativeGains = new double[3];
   private double[] positionIntegralGains = new double[3];
   private double positionMaxIntegralError = 0.0;

   private double positionMaximumAcceleration = Double.POSITIVE_INFINITY;
   private double positionMaximumJerk = Double.POSITIVE_INFINITY;

   @Override
   public void set(PositionPIDGainsInterface gains)
   {
      for (int i = 0; i < 3; i++)
      {
         positionProportionalGains[i] = gains.getProportionalGains()[i];
         positionDerivativeGains[i] = gains.getDerivativeGains()[i];
         positionIntegralGains[i] = gains.getIntegralGains()[i];
      }

      positionMaxIntegralError = gains.getMaximumIntegralError();
      positionMaximumAcceleration = gains.getMaximumAcceleration();
      positionMaximumJerk = gains.getMaximumJerk();
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
      this.positionProportionalGains[0] = proportionalGainX;
      this.positionProportionalGains[1] = proportionalGainY;
      this.positionProportionalGains[2] = proportionalGainZ;
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      this.positionDerivativeGains[0] = derivativeGainX;
      this.positionDerivativeGains[1] = derivativeGainY;
      this.positionDerivativeGains[2] = derivativeGainZ;
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      this.positionIntegralGains[0] = integralGainX;
      this.positionIntegralGains[1] = integralGainY;
      this.positionIntegralGains[2] = integralGainZ;
      this.positionMaxIntegralError = maxIntegralError;
   }

   public void setMaximumAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      positionMaximumAcceleration = maxAcceleration;
      positionMaximumJerk = maxJerk;
   }

   @Override
   public double[] getProportionalGains()
   {
      return positionProportionalGains;
   }

   @Override
   public double[] getDerivativeGains()
   {
      return positionDerivativeGains;
   }

   @Override
   public double[] getIntegralGains()
   {
      return positionIntegralGains;
   }

   @Override
   public double getMaximumIntegralError()
   {
      return positionMaxIntegralError;
   }

   @Override
   public double getMaximumAcceleration()
   {
      return positionMaximumAcceleration;
   }

   @Override
   public double getMaximumJerk()
   {
      return positionMaximumJerk;
   }
}
