package us.ihmc.wholeBodyControlCore.pidGains;

public interface IntegratorGainsBasics extends IntegratorGainsReadOnly
{
   void setKi(double ki);

   void setMaxIntegralError(double maxIntegralError);

   void setIntegralLeakRatio(double integralLeakRatio);
}
