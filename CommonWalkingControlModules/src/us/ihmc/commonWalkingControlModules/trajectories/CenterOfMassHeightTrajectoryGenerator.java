package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;


public interface CenterOfMassHeightTrajectoryGenerator
{
   public abstract void initialize(RobotSide supportLeg);
   public void solve(CenterOfMassHeightOutputData centerOfMassHeightOutputDataToPack, CenterOfMassHeightInputData centerOfMassHeightInputData);
}
