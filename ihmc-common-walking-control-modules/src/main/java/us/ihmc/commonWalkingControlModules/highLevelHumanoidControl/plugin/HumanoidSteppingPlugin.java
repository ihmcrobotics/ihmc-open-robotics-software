package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;

public interface HumanoidSteppingPlugin extends HighLevelHumanoidControllerPlugin
{
   void setFootstepAdjustment(FootstepAdjustment footstepAdjustment);
}
