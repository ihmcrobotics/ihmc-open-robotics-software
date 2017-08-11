package us.ihmc.robotics.controllers;

public class PIDGains extends PDGains
{
   private double ki;
   private double maxIntegralError;
   private double integralLeakRatio;

   public PIDGains()
   {
      super();
   }

   public PIDGains(String name)
   {
      super(name);
   }

   public double getKi()
   {
      return ki;
   }

   public void setKi(double ki)
   {
      this.ki = ki;
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
