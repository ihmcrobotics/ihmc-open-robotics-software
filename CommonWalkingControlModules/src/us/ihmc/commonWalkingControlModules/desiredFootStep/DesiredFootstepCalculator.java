package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredFootstepCalculator
{
   public abstract FootstepDataMessage updateAndGetDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide);

   public abstract FootstepDataMessage predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, FootstepDataMessage desiredFootstep, double timeFromNow);

   public abstract void initialize();

   public abstract boolean isDone();
}
