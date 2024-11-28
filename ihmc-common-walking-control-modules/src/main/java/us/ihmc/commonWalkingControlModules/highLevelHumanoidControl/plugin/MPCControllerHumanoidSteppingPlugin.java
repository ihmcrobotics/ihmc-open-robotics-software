package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;

public interface MPCControllerHumanoidSteppingPlugin extends HighLevelHumanoidMPCControllerPlugin
{
   void setFootstepAdjustment(FootstepAdjustment footstepAdjustment);
}
