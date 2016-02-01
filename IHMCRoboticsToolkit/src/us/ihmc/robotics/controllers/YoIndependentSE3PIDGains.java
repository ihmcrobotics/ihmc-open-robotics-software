package us.ihmc.robotics.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class YoIndependentSE3PIDGains implements YoSE3PIDGains
{
   private final YoEuclideanPositionGains positionGains;
   private final YoAxisAngleOrientationGains orientationGains;

   public YoIndependentSE3PIDGains(String prefix, YoVariableRegistry registry)
   {
      positionGains = new YoEuclideanPositionGains(prefix, registry);
      orientationGains = new YoAxisAngleOrientationGains(prefix, registry);
   }

   public void reset()
   {
      positionGains.reset();
      orientationGains.reset();
   }

   public YoEuclideanPositionGains getPositionGains()
   {
      return positionGains;
   }

   public YoAxisAngleOrientationGains getOrientationGains()
   {
      return orientationGains;
   }

   public void setPositionProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      positionGains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setPositionDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      positionGains.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setPositionIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      positionGains.setIntegralGains(integralGainX, integralGainY, integralGainZ, maxIntegralError);
   }

   public void setPositionProportionalGains(double[] proportionalGains)
   {
      positionGains.setProportionalGains(proportionalGains);
   }

   public void setPositionDerivativeGains(double[] derivativeGains)
   {
      positionGains.setDerivativeGains(derivativeGains);
   }

   public void setPositionIntegralGains(double[] integralGains, double maxIntegralError)
   {
      positionGains.setIntegralGains(integralGains, maxIntegralError);
   }

   public void setPositionMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      positionGains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void setOrientationProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      orientationGains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setOrientationDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      orientationGains.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setOrientationIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      orientationGains.setIntegralGains(integralGainX, integralGainY, integralGainZ, maxIntegralError);
   }

   public void setOrientationProportionalGains(double[] proportionalGains)
   {
      orientationGains.setProportionalGains(proportionalGains);
   }

   public void setOrientationDerivativeGains(double[] derivativeGains)
   {
      orientationGains.setDerivativeGains(derivativeGains);
   }

   public void setOrientationIntegralGains(double[] integralGains, double maxIntegralError)
   {
      orientationGains.setIntegralGains(integralGains, maxIntegralError);
   }

   public void setOrientationMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      orientationGains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }
}
