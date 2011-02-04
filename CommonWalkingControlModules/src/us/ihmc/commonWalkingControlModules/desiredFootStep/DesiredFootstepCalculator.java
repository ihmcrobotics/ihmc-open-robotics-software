package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.RobotSide;

public interface DesiredFootstepCalculator
{
   public abstract Footstep getDesiredFootstep();

   public abstract void updateDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide);

}
