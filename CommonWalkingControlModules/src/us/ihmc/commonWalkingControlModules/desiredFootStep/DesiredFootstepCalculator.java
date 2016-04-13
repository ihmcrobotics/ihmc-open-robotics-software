package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataControllerCommand;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredFootstepCalculator
{
   public abstract FootstepDataControllerCommand updateAndGetDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide);

   public abstract FootstepDataControllerCommand predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, FootstepDataControllerCommand desiredFootstep, double timeFromNow);

   public abstract void initialize();

   public abstract boolean isDone();
}
