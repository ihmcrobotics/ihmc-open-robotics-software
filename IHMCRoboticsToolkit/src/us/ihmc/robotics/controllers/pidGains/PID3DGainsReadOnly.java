package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;

public interface PID3DGainsReadOnly
{
   public abstract Matrix3DReadOnly getProportionalGainMatrix();

   public abstract Matrix3DReadOnly getDerivativeGainMatrix();

   public abstract Matrix3DReadOnly getIntegralGainMatrix();

   public abstract double[] getProportionalGains();

   public abstract double[] getDerivativeGains();

   public abstract double[] getIntegralGains();

   public abstract double getMaximumIntegralError();

   public abstract double getMaximumDerivativeError();

   public abstract double getMaximumProportionalError();

   public abstract double getMaximumFeedback();

   public abstract double getMaximumFeedbackRate();
}
