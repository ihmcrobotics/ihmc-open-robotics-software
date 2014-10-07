package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

public interface DesiredFootstepAdjustor
{
   public abstract Footstep adjustDesiredFootstep(Footstep stanceFootstep, Footstep baseSwingFootstep);
}
