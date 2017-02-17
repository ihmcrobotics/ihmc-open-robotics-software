package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public interface YoPositionPIDGainsInterface extends PositionPIDGainsInterface
{
   public abstract void reset();

   public abstract Matrix3DReadOnly createProportionalGainMatrix();

   public abstract Matrix3DReadOnly createDerivativeGainMatrix();

   public abstract Matrix3DReadOnly createIntegralGainMatrix();

   public abstract void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ);

   public abstract void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ);

   public abstract void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError);

   public abstract void setProportionalGains(double[] proportionalGains);

   public abstract void setDerivativeGains(double[] derivativeGains);

   public abstract void setIntegralGains(double[] integralGains, double maxIntegralError);

   public abstract void setTangentialDampingGains(TangentialDampingGains tangentialDampingGains);

   public abstract void setTangentialDampingGains(double kdReductionRatio, double parallelDampingDeadband, double positionErrorForMinKd);

   public abstract void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate);

   public abstract void setMaxDerivativeError(double maxDerivativeError);

   public abstract void setMaxProportionalError(double maxProportionalError);

   public abstract DoubleYoVariable getYoMaximumFeedback();

   public abstract DoubleYoVariable getYoMaximumFeedbackRate();

   public abstract DoubleYoVariable getYoMaximumDerivativeError();

   public abstract DoubleYoVariable getYoMaximumProportionalError();

   public abstract YoTangentialDampingGains getYoTangentialDampingGains();
}
