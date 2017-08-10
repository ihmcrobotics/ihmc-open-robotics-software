package us.ihmc.robotics.controllers;

public class PIDGains
{
   private final String name;

   private double kp;
   private double kd;
   private double ki;
   private double zeta;
   private double maximumOutput;
   private double maximumFeedback;
   private double maximumFeedbackRate;
   private double positionDeadband;
   private double maxIntegralError;
   private double integralLeakRatio;

   public PIDGains()
   {
      this("");
   }

   public PIDGains(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public double getKp()
   {
      return kp;
   }

   public void setKp(double kp)
   {
      this.kp = kp;
      kd = GainCalculator.computeDerivativeGain(kp, zeta);
   }

   public double getZeta()
   {
      return zeta;
   }

   public void setZeta(double zeta)
   {
      this.zeta = zeta;
      kd = GainCalculator.computeDerivativeGain(kp, zeta);
   }

   public double getKd()
   {
      return kd;
   }

   public double getKi()
   {
      return ki;
   }

   public void setKi(double ki)
   {
      this.ki = ki;
   }

   public double getMaximumOutput()
   {
      return maximumOutput;
   }

   public void setMaximumOutput(double maximumOutput)
   {
      this.maximumOutput = maximumOutput;
   }

   public double getMaximumFeedback()
   {
      return maximumFeedback;
   }

   public void setMaximumFeedback(double maximumFeedback)
   {
      this.maximumFeedback = maximumFeedback;
   }

   public double getMaximumFeedbackRate()
   {
      return maximumFeedbackRate;
   }

   public void setMaximumFeedbackRate(double maximumFeedbackRate)
   {
      this.maximumFeedbackRate = maximumFeedbackRate;
   }

   public double getPositionDeadband()
   {
      return positionDeadband;
   }

   public void setPositionDeadband(double positionDeadband)
   {
      this.positionDeadband = positionDeadband;
   }

   public double getMaxIntegralError()
   {
      return maxIntegralError;
   }

   public void setMaxIntegralError(double maxIntegralError)
   {
      this.maxIntegralError = maxIntegralError;
   }

   public double getIntegralLeakRatio()
   {
      return integralLeakRatio;
   }

   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      this.integralLeakRatio = integralLeakRatio;
   }
}
