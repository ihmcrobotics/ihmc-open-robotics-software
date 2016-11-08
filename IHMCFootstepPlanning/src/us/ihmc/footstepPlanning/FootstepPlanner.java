package us.ihmc.footstepPlanning;

import java.util.List;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

public interface FootstepPlanner
{
   /**
    * Set the initial stance side and the pose of the initial stance foot for planning.
    * @param stanceFootPose      Pose of the sole frame of the initial stance foot
    * @param side                RobotSide of the initial stance foot
    */
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side);

   /**
    * Set the goal pose of the robot (middle of the sole frames)
    * @param goalPose            Goal pose for footstep plan
    */
   public void setGoalPose(FramePose goalPose);

   /**
    * Set possible stepping regions for the footstep planner. If null is passed the
    * planner will assume flat ground.
    * @param planarRegionsList   List of planar regions that describe the environment
    */
   public void setPlanarRegions(PlanarRegionsList planarRegionsList);

   /**
    * Plan a sequence of footsteps given initial and goal conditions. The poses describe
    * the location and orientation of the sole frame in world.
    * @return footstepPlan       Sequence of footsteps from initial stance foot to goal
    */
   public FootstepPlanningResult plan(List<FramePose> solePosesToPack);
}
