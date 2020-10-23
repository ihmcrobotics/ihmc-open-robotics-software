package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PDControllerWithGainSetter extends AbstractPDController
{
   private PDGainsReadOnly gains;

   public PDControllerWithGainSetter(String suffix, YoRegistry registry)
   {
      super(suffix, registry);
   }

   public void setGains(PDGainsReadOnly gains)
   {
      this.gains = gains;
   }

   @Override
   public double getProportionalGain()
   {
      return gains.getKp();
   }

   @Override
   public double getDerivativeGain()
   {
      return gains.getKd();
   }

   @Override
   public double getPositionDeadband()
   {
      return gains.getPositionDeadband();
   }

}
