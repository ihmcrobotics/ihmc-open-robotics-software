package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

public interface SwingStateInterface
{

   public abstract void setFootstep(Footstep footstep, TrajectoryParameters trajectoryParameters, boolean useLowHeightTrajectory);

   public abstract void replanTrajectory(Footstep footstep, double swingTimeRemaining, boolean useLowHeightTrajectory);

}