package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;

/*
 * Considering articulated trajectory should be added in near future.
 * 170627 Inho Lee.
 */

public interface EndEffectorTrajectory
{  
   public double getTrajectoryTime();
   public Pose getEndEffectorPose(double time);
   public RobotSide getRobotSide();
   public RobotSide getAnotherRobotSide();
}
