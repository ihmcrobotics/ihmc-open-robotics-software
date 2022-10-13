package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;

public interface SteppingPlugin extends HighLevelHumanoidControllerPlugin
{
   void setFootstepAdjustment(FootstepAdjustment footstepAdjustment);
}
