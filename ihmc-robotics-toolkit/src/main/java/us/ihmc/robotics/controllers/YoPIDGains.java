package us.ihmc.robotics.controllers;

import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoPIDGains extends YoPDGains
{
   protected final YoDouble ki;
   private final YoDouble maxIntegralError, integralLeakRatio;

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

   public void set(PIDGains pidGains)
   {
      super.set(pidGains);
      setKi(pidGains.getKi());
      setMaximumIntegralError(pidGains.getMaxIntegralError());
      setIntegralLeakRatio(pidGains.getIntegralLeakRatio());
   }
}
