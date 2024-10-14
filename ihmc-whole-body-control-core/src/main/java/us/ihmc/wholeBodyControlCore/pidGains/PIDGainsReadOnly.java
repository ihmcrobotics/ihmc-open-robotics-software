package us.ihmc.wholeBodyControlCore.pidGains;

public interface PIDGainsReadOnly extends PDGainsReadOnly, IntegratorGainsReadOnly
{
   default boolean equals(PIDGainsReadOnly other)
   {
      if (!PDGainsReadOnly.super.equals(other))
         return false;
      if (!IntegratorGainsReadOnly.super.equals(other))
         return false;
      return true;
   }
}
