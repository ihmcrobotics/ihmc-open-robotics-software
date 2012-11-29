package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.robotSide.RobotSide;


public interface DesiredFootstepAdjustor
{
   public abstract Footstep adjustDesiredFootstep(Footstep baseFootstep, RobotSide swingLegSide);
}
