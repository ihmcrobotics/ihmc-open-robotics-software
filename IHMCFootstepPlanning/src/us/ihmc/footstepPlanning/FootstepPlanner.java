package us.ihmc.footstepPlanning;

import java.util.List;

import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface FootstepPlanner
{
   public void setInitialStanceFoot(FramePose2d stanceFootPose, RobotSide side);

   public void setGoalPose(FramePose2d goalPose);

//   public void setPlanarRegions(); // TODO

   public List<FramePose2d> plan();
}
