package us.ihmc.robotics.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoPIDGains extends YoPDGains
{
   private final DoubleYoVariable ki, maxIntegralError;

   public YoPIDGains(String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);

      ki = new DoubleYoVariable("ki" + suffix, registry);
      maxIntegralError = new DoubleYoVariable("maxIntegralError" + suffix, registry);
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

   public DoubleYoVariable getYoKi()
   {
      return ki;
   }

   public DoubleYoVariable getYoMaxIntegralError()
   {
      return maxIntegralError;
   }
}
