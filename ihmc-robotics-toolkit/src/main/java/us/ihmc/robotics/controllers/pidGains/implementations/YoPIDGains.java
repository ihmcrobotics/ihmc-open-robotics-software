package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoPIDGains extends YoPDGains implements PIDGainsReadOnly
{
   private final YoDouble ki;
   private final YoDouble maxIntegralError;
   private final YoDouble integralLeakRatio;

   public YoPIDGains(String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);

      ki = new YoDouble("ki" + suffix, registry);
      maxIntegralError = new YoDouble("maxIntegralError" + suffix, registry);
      integralLeakRatio = new YoDouble("integralLeakRatio" + suffix, registry);
      integralLeakRatio.set(1.0);
   }

   public void setPIDGains(double kp, double zeta, double ki, double maxIntegralError)
   {
      setPDGains(kp, zeta);
      this.ki.set(ki);
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setKi(double ki)
   {
      this.ki.set(ki);
   }

   public void setMaximumIntegralError(double maxIntegralError)
   {
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      this.integralLeakRatio.set(MathTools.clamp(integralLeakRatio, 0.0, 1.0));
   }

   @Override
   public double getKi()
   {
      return ki.getDoubleValue();
   }

   @Override
   public double getMaxIntegralError()
   {
      return maxIntegralError.getDoubleValue();
   }

   @Override
   public double getIntegralLeakRatio()
   {
      return integralLeakRatio.getDoubleValue();
   }

   public YoDouble getYoKi()
   {
      return ki;
   }

   public YoDouble getYoMaxIntegralError()
   {
      return maxIntegralError;
   }

   public YoDouble getYoIntegralLeakRatio()
   {
      return integralLeakRatio;
   }

   public void set(YoPIDGains other)
   {
      super.set(other);
      ki.set(other.ki.getDoubleValue());
      maxIntegralError.set(other.maxIntegralError.getDoubleValue());
      integralLeakRatio.set(other.integralLeakRatio.getDoubleValue());
   }

   public void set(PIDGainsReadOnly pidGains)
   {
      super.set(pidGains);
      setKi(pidGains.getKi());
      setMaximumIntegralError(pidGains.getMaxIntegralError());
      setIntegralLeakRatio(pidGains.getIntegralLeakRatio());
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
