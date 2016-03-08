package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootstepDataControllerCommand;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredFootstepCalculator
{
   public abstract FootstepDataControllerCommand updateAndGetDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide);

   public abstract FootstepDataControllerCommand predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, FootstepDataControllerCommand desiredFootstep);

   public abstract void initialize();

   public abstract boolean isDone();
}
