package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface HeadingAndVelocityScriptFactory
{
   HeadingAndVelocityScript getHeadingAndVelocityScript(double controlDT, DoubleProvider timeProvider, YoRegistry parentRegistry);
}
