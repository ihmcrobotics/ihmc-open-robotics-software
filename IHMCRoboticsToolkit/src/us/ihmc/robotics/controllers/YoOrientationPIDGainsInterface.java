package us.ihmc.robotics.controllers;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public interface YoOrientationPIDGainsInterface extends OrientationPIDGainsInterface
{
   public abstract Matrix3d createProportionalGainMatrix();

   public abstract Matrix3d createDerivativeGainMatrix();

   public abstract Matrix3d createIntegralGainMatrix();

   public abstract void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ);

   public abstract void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ);

   public abstract void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError);

   public abstract void setProportionalGains(double[] proportionalGains);

   public abstract void setDerivativeGains(double[] derivativeGains);

   public abstract void setIntegralGains(double[] integralGains, double maxIntegralError);

   public abstract void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk);

   public abstract DoubleYoVariable getYoMaximumAcceleration();

   public abstract DoubleYoVariable getYoMaximumJerk();
}
