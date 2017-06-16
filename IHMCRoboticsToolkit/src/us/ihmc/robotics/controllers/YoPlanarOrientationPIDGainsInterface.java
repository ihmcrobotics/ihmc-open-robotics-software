package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.yoVariables.variable.YoDouble;

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

   public abstract YoDouble getYoMaximumFeedback();

   public abstract YoDouble getYoMaximumFeedbackRate();

   public abstract YoDouble getYoMaximumDerivativeError();

   public abstract YoDouble getYoMaximumProportionalError();
}
