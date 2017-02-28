package us.ihmc.robotics.controllers;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoPIDGains extends YoPDGains
{
   protected final DoubleYoVariable ki;
   private final DoubleYoVariable maxIntegralError, integralLeakRatio;

   public YoPIDGains(String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);

      ki = new DoubleYoVariable("ki" + suffix, registry);
      maxIntegralError = new DoubleYoVariable("maxIntegralError" + suffix, registry);
      integralLeakRatio = new DoubleYoVariable("integralLeakRatio" + suffix, registry);
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

   public DoubleYoVariable getYoKi()
   {
      return ki;
   }

   public DoubleYoVariable getYoMaxIntegralError()
   {
      return maxIntegralError;
   }

   public DoubleYoVariable getYoIntegralLeakRatio()
   {
      return integralLeakRatio;
   }
}
