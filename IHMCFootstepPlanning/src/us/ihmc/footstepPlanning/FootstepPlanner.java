package us.ihmc.footstepPlanning;

import java.util.List;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;

public interface FootstepPlanner
{
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side);

   public void setGoalPose(FramePose goalPose);

   public List<FramePose> plan();
}
