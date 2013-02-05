package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector2d;


public interface CenterOfMassHeightTrajectoryGenerator
{
   public abstract void initialize(RobotSide supportLeg);

   public abstract void compute();

   public void solve(CenterOfMassHeightOutputData centerOfMassHeightOutputDataToPack, CenterOfMassHeightInputData centerOfMassHeightInputData);

}
