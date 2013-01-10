package us.ihmc.commonWalkingControlModules.desiredFootStep;

public interface DesiredFootstepAdjustor
{
   public abstract Footstep adjustDesiredFootstep(Footstep stanceFootstep, Footstep baseSwingFootstep);
}
