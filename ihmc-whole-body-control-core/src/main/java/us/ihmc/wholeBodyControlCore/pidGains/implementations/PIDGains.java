package us.ihmc.wholeBodyControlCore.pidGains.implementations;

import us.ihmc.wholeBodyControlCore.pidGains.PIDGainsBasics;
import us.ihmc.wholeBodyControlCore.pidGains.PIDGainsReadOnly;

public class PIDGains extends PDGains implements PIDGainsBasics
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

   @Override
   public void setMaxIntegralError(double maxIntegralError)
   {
      this.maxIntegralError = maxIntegralError;
   }

   @Override
   public double getIntegralLeakRatio()
   {
      return integralLeakRatio;
   }

   @Override
   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      this.integralLeakRatio = integralLeakRatio;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PIDGainsReadOnly)
         return PIDGainsBasics.super.equals((PIDGainsReadOnly) object);
      else
         return false;
   }
}
