package us.ihmc.robotics.controllers.pidGains;

public interface PIDGainsReadOnly extends PDGainsReadOnly, IntegratorGainsReadOnly
{
   default boolean equals(PIDGainsReadOnly other)
   {
      if (!PDGainsReadOnly.super.equals(other))
         return false;
      if (getKi() != other.getKi())
         return false;
      if (getMaxIntegralError() != other.getMaxIntegralError())
         return false;
      if (getIntegralLeakRatio() != other.getIntegralLeakRatio())
         return false;
      return true;
   }
}
