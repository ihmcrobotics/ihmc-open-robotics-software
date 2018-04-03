package us.ihmc.commonWalkingControlModules.desiredFootStep;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredFootstepCalculator
{
   public abstract FootstepDataMessage updateAndGetDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide, double stepDuration);

   public abstract FootstepDataMessage predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, FootstepDataMessage desiredFootstep, double timeFromNow,
         double stepDuration);

   public abstract void initialize();

   public abstract boolean isDone();
}
