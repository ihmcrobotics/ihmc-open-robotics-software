package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.humanoidRobotics.footstep.Footstep;

public interface DesiredFootstepAdjustor
{
   public abstract Footstep adjustDesiredFootstep(Footstep stanceFootstep, Footstep baseSwingFootstep);
}
