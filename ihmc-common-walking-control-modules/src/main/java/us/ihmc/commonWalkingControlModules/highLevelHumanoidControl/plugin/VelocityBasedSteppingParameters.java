package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

public interface VelocityBasedSteppingParameters
{
   double getMaxDesiredForwardVelocity();

   double getMaxDesiredBackwardVelocity();

   double getMaxDesiredLateralVelocity();

   double getMaxDesiredTurningVelocity();
}
