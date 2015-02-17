package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;

public interface SwingStateInterface
{

   public abstract void setFootstep(Footstep footstep, boolean useLowHeightTrajectory);

   public abstract void replanTrajectory(Footstep footstep, double swingTimeRemaining, boolean useLowHeightTrajectory);

   public abstract void setInitialDesireds(FrameOrientation initialOrientation, FrameVector initialAngularVelocity);

}