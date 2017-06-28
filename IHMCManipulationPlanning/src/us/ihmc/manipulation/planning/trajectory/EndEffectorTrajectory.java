package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.robotics.geometry.transformables.Pose;

/*
 * Considering articulated trajectory should be added in near future.
 * 170627 Inho Lee.
 */

public interface EndEffectorTrajectory
{   
   public double getTrajectoryTime();
   public Pose getEndEffectorPose(double time);
}
