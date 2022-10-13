package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public interface VelocityBasedSteppingParameters
{
   double getMaxDesiredForwardVelocity();

   double getMaxDesiredBackwardVelocity();

   double getMaxDesiredLateralVelocity();

   double getMaxDesiredTurningVelocity();
}
