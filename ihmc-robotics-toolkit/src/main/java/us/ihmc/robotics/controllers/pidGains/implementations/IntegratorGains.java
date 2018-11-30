package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.IntegratorGainsReadOnly;

public class IntegratorGains implements IntegratorGainsReadOnly
{
   private double ki = 0.0;
   private double maxIntegralError = Double.POSITIVE_INFINITY;
   private double integralLeakRatio = 1.0;

   @Override
   public double getKi()
   {
      return ki;
   }

   public void setKi(double ki)
   {
      this.ki = ki;
   }

   @Override
   public double getMaxIntegralError()
   {
      return maxIntegralError;
   }

   public void setMaxIntegralError(double maxIntegralError)
   {
      this.maxIntegralError = maxIntegralError;
   }

   @Override
   public double getIntegralLeakRatio()
   {
      return integralLeakRatio;
   }

   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      this.integralLeakRatio = integralLeakRatio;
   }
}
