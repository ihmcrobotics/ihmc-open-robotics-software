package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.utilities.humanoidRobot.footstep.Footstep;

public interface DesiredFootstepAdjustor
{
   public abstract Footstep adjustDesiredFootstep(Footstep stanceFootstep, Footstep baseSwingFootstep);
}
