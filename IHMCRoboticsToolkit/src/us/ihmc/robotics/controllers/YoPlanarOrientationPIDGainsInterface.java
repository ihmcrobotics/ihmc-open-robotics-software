package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public interface YoPlanarOrientationPIDGainsInterface extends OrientationPIDGainsInterface
{
   public abstract RotationMatrix createProportionalGainMatrix();

   public abstract RotationMatrix createDerivativeGainMatrix();

   public abstract RotationMatrix createIntegralGainMatrix();

   public abstract void setProportionalGains(double proportionalGainX);

   public abstract void setDerivativeGains(double derivativeGainX);

   public abstract void setIntegralGains(double integralGainX, double maxIntegralError);

   public abstract void setProportionalGains(double[] proportionalGains);

   public abstract void setDerivativeGains(double[] derivativeGains);

   public abstract void setIntegralGains(double[] integralGains, double maxIntegralError);

   public abstract void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate);

   public abstract void setMaxDerivativeError(double maxDerivativeError);

   public abstract void setMaxProportionalError(double maxProportionalError);

   public abstract DoubleYoVariable getYoMaximumFeedback();

   public abstract DoubleYoVariable getYoMaximumFeedbackRate();

   public abstract DoubleYoVariable getYoMaximumDerivativeError();

   public abstract DoubleYoVariable getYoMaximumProportionalError();
}
