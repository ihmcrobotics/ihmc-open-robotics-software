package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;

public class PIDGains extends PDGains implements PIDGainsReadOnly
{
   private double ki;
   private double maxIntegralError;
   private double integralLeakRatio;

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

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PIDGainsReadOnly)
         return PIDGainsReadOnly.super.equals((PIDGainsReadOnly) object);
      else
         return false;
   }
}
