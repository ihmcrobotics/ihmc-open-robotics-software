package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;

public interface GroundTrajectoryGenerator
{
   public void getViaPoints(YoFramePoint[] yoFramePoints, RobotSide swingSide, double tStart, FramePoint startPointIn, double tEnd, FramePoint endPointIn, double[] tOfViaPoints, double heightOfViaPoints[]);

}