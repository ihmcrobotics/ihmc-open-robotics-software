package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import us.ihmc.robotics.geometry.transformables.Pose;

public interface ConstrainedEndEffectorTrajectory
{   
   public double getTrajectoryTime();
   public Pose getEndEffectorPose(double time);
}
