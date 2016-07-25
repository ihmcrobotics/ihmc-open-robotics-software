package us.ihmc.robotics.controllers;

public interface OrientationPIDGainsInterface
{
   public abstract void set(OrientationPIDGainsInterface gains);

   public abstract double[] getProportionalGains();

   public abstract double[] getDerivativeGains();

   public abstract double[] getIntegralGains();

   public abstract double getMaximumIntegralError();

   public abstract double getMaximumDerivativeError();

   public abstract double getMaximumProportionalError();

   public abstract double getMaximumFeedback();

   public abstract double getMaximumFeedbackRate();
}