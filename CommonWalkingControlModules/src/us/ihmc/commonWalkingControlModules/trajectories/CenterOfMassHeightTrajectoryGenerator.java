package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotSide.RobotSide;


public interface CenterOfMassHeightTrajectoryGenerator
{
   public abstract void initialize(RobotSide supportLeg);

   public abstract void compute();

   public abstract double getDesiredCenterOfMassHeight();

   public abstract double getDesiredCenterOfMassHeightSlope();

   public abstract double getDesiredCenterOfMassHeightSecondDerivative();
}
