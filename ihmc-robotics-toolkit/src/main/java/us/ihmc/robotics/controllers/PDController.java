package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PDController extends AbstractPDController
{
   private final YoDouble proportionalGain;
   private final YoDouble derivativeGain;
   private final YoDouble positionDeadband;

   public PDController(String suffix, YoRegistry registry)
   {
      this(new YoDouble("kp_" + suffix, registry),
           new YoDouble("kd_" + suffix, registry),
           new YoDouble("positionDeadband_" + suffix, registry),
           suffix,
           registry);

      proportionalGain.set(0.0);
      derivativeGain.set(0.0);
      positionDeadband.set(0.0);
   }

   public PDController(YoDouble proportionalGain, YoDouble derivativeGain, String suffix, YoRegistry registry)
   {
      this(proportionalGain, derivativeGain, new YoDouble("positionDeadband_" + suffix, registry), suffix, registry);

      positionDeadband.set(0.0);
   }

   public PDController(YoPDGains pdGains, String suffix, YoRegistry registry)
   {
      this(pdGains.getYoKp(), pdGains.getYoKd(), pdGains.getYoPositionDeadband(), suffix, registry);
   }

   public PDController(YoDouble proportionalGain, YoDouble derivativeGain, YoDouble positionDeadband, String suffix, YoRegistry registry)
   {
      super(proportionalGain, derivativeGain, positionDeadband, suffix, registry);

      this.proportionalGain = (YoDouble) super.proportionalGain;
      this.derivativeGain = (YoDouble) super.derivativeGain;
      this.positionDeadband = (YoDouble) super.positionDeadband;
   }

   public void setGains(PDGainsReadOnly gains)
   {
      setProportionalGain(gains.getKp());
      setDerivativeGain(gains.getKd());
      setPositionDeadband(gains.getPositionDeadband());
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
