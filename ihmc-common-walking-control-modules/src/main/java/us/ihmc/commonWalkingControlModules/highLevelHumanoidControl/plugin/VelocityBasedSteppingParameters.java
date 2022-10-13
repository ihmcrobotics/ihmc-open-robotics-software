package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class VelocityBasedSteppingParameters
{
   private final YoDouble maxDesiredForwardVelocity;
   private final YoDouble maxDesiredBackwardVelocity;
   private final YoDouble maxDesiredLateralVelocity;

   private final YoDouble maxDesiredTurningVelocity;

   public VelocityBasedSteppingParameters(String suffix, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      maxDesiredForwardVelocity = new YoDouble("maxDesiredForwardVelocity" + suffix, registry);
      maxDesiredBackwardVelocity = new YoDouble("maxDesiredBackwardVelocity" + suffix, registry);
      maxDesiredLateralVelocity = new YoDouble("maxDesiredLateralVelocity" + suffix, registry);

      maxDesiredTurningVelocity = new YoDouble("maxDesiredTurningVelocity" + suffix, registry);

      parentRegistry.addChild(registry);
   }

   public double getMaxDesiredForwardVelocity()
   {
      return maxDesiredForwardVelocity.getDoubleValue();
   }

   public double getMaxDesiredBackwardVelocity()
   {
      return maxDesiredBackwardVelocity.getDoubleValue();
   }

   public double getMaxDesiredLateralVelocity()
   {
      return maxDesiredLateralVelocity.getDoubleValue();
   }

   public double getMaxDesiredTurningVelocity()
   {
      return maxDesiredTurningVelocity.getDoubleValue();
   }

   public void setMaxDesiredForwardVelocity(double maxVelocity)
   {
      maxDesiredForwardVelocity.set(maxVelocity);
   }

   public void setMaxDesiredBackwardVelocity(double maxVelocity)
   {
      maxDesiredBackwardVelocity.set(maxVelocity);
   }

   public void setMaxDesiredLateralVelocity(double maxVelocity)
   {
      maxDesiredLateralVelocity.set(maxVelocity);
   }

   public void setMaxDesiredTurningVelocity(double maxVelocity)
   {
      maxDesiredTurningVelocity.set(maxVelocity);
   }
}
