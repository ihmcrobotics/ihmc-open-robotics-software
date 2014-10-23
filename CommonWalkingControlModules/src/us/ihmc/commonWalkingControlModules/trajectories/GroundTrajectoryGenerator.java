package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public interface GroundTrajectoryGenerator
{
   public void getViaPoints(YoFramePoint[] yoFramePoints, RobotSide swingSide, double tStart, FramePoint startPointIn, double tEnd, FramePoint endPointIn, double[] tOfViaPoints, double heightOfViaPoints[]);

}