package us.ihmc.robotics.controllers;

public interface PositionPIDGainsInterface
{
   public abstract void set(PositionPIDGainsInterface gains);

   public abstract double[] getProportionalGains();

   public abstract double[] getDerivativeGains();

   public abstract double[] getIntegralGains();

   public abstract double getMaximumIntegralError();

   public abstract double getMaximumDerivativeError();

   public abstract double getMaximumProportionalError();

   public abstract double getMaximumFeedback();

   public abstract double getMaximumFeedbackRate();

   public abstract TangentialDampingGains getTangentialDampingGains();
}