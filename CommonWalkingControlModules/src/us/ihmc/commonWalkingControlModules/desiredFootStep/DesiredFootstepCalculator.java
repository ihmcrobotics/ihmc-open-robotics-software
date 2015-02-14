package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;

public interface DesiredFootstepCalculator
{
   public abstract Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide);
   
   public abstract Footstep predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, Footstep desiredFootstep);

   public abstract void initialize();

   public abstract boolean isDone();
}
