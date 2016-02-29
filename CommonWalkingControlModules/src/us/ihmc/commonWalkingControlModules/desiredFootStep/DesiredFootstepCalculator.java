package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredFootstepCalculator
{
   public abstract ModifiableFootstepDataMessage updateAndGetDesiredFootstep(RobotSide supportLegSide);

   public abstract void initializeDesiredFootstep(RobotSide supportLegSide);

   public abstract ModifiableFootstepDataMessage predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, ModifiableFootstepDataMessage desiredFootstep);

   public abstract void initialize();

   public abstract boolean isDone();
}
