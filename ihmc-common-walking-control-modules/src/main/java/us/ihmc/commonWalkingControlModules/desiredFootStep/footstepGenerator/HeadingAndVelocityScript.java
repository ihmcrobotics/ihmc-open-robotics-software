package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class HeadingAndVelocityScript implements Updatable
{
   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public abstract DesiredVelocityProvider getDesiredVelocityProvider();

   public abstract DesiredTurningVelocityProvider getDesiredTurningVelocityProvider();

   public abstract void update(double time);
}
