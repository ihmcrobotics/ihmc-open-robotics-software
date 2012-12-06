package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.robotSide.RobotSide;

public interface DesiredFootstepCalculator
{
   public abstract Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide);
}
