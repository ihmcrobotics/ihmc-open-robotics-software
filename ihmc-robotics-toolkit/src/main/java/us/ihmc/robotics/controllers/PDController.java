package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PDController extends AbstractPDController
{
   private final YoDouble proportionalGain;
   private final YoDouble derivativeGain;
   private final YoDouble positionDeadband;

   public PDController(String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      proportionalGain = new YoDouble("kp_" + suffix, registry);
      proportionalGain.set(0.0);

      derivativeGain = new YoDouble("kd_" + suffix, registry);
      derivativeGain.set(0.0);

      positionDeadband = new YoDouble("positionDeadband_" + suffix, registry);
      positionDeadband.set(0.0);
   }

   public PDController(YoDouble proportionalGain, YoDouble derivativeGain, String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;

      positionDeadband = new YoDouble("positionDeadband_" + suffix, registry);
      positionDeadband.set(0.0);
   }

   public PDController(YoDouble proportionalGain, YoDouble derivativeGain, YoDouble positionDeadband, String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      this.proportionalGain = proportionalGain;
      this.derivativeGain = derivativeGain;
      this.positionDeadband = positionDeadband;
   }

   public PDController(YoPDGains pdGains, String suffix, YoVariableRegistry registry)
   {
      super(suffix, registry);
      this.proportionalGain = pdGains.getYoKp();
      this.derivativeGain = pdGains.getYoKd();
      this.positionDeadband = pdGains.getYoPositionDeadband();
   }

   @Override
   public double getProportionalGain()
   {
      return proportionalGain.getValue();
   }

   @Override
   public double getDerivativeGain()
   {
      return derivativeGain.getValue();
   }

   @Override
   public double getPositionDeadband()
   {
      return positionDeadband.getValue();
   }

   public void setProportionalGain(double proportionalGain)
   {
      this.proportionalGain.set(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      this.derivativeGain.set(derivativeGain);
   }

   public void setPositionDeadband(double positionDeadband)
   {
      this.positionDeadband.set(positionDeadband);
   }
}
