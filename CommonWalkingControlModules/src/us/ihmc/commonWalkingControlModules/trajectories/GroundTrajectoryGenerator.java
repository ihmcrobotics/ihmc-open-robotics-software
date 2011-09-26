package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface GroundTrajectoryGenerator
{
   public void getViaPoints(FramePoint[] viaPointsToPack, RobotSide swingSide, double tStart, FramePoint startPointIn, double tEnd, FramePoint endPointIn, FrameVector initialVelocityIn, double[] tOfViaPoints, double heightOfViaPoints[]);
}
