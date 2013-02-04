package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector2d;


public interface CenterOfMassHeightTrajectoryGenerator
{
   public abstract void initialize(RobotSide supportLeg);

   public abstract void compute();

   public abstract double getDesiredCenterOfMassHeight();

   public abstract FrameVector2d getDesiredCenterOfMassHeightSlope();

   public abstract FrameVector2d getDesiredCenterOfMassHeightSecondDerivative();
}
