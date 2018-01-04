package us.ihmc.footstepPlanning;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface FootstepPlanner
{
   /**
    * Set the initial stance side and the pose of the initial stance foot for planning.
    * @param stanceFootPose      Pose of the sole frame of the initial stance foot
    * @param side                RobotSide of the initial stance foot
    */
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side);
   
   /**
    * Sets the initial foot poses at the start of the plan. The planner will determine the
    * best first step side.
    */
   public default void setFootPoses(SideDependentList<FramePose> footPoses)
   {
      setFootPoses(footPoses, null);
   }
   
   /**
    * Sets the initial foot poses at the start of the plan. A initial stance side can be specified
    * if it is required that the first step is taken with a specific robot side. If the initial
    * side is set to null the planner will determine the best first step side.
    */
   public default void setFootPoses(SideDependentList<FramePose> footPoses, RobotSide initialSanceSide)
   {
      setInitialStanceFoot(footPoses.get(initialSanceSide), initialSanceSide);
   }

   /**
    * Set the FootstepPlannerGoal of the robot.
    * @param goal FootstepPlannerGoal indicating the goal region, footstep, or other options.
    */
   public void setGoal(FootstepPlannerGoal goal);

   /**
    * Sets a timeout after which the planner will give up if no solution was found. By default this
    * is set to infinity.
    * @param timeout timeout of the planner.
    */
   public void setTimeout(double timeout);

   /**
    * Set possible stepping regions for the footstep planner. If null is passed the
    * planner will assume flat ground.
    * @param planarRegionsList   List of planar regions that describe the environment
    */
   public void setPlanarRegions(PlanarRegionsList planarRegionsList);

   /**
    * Plan a sequence of footsteps given initial and goal conditions. The poses describe
    * the location and orientation of the sole frame in world.
    * @return FootstepPlanningResult   Whether or not the plan succeeded.
    */
   public FootstepPlanningResult plan();

   /**
    * Returns the plan that was solved for during the plan() method.
    *
    * @return
    */
   public FootstepPlan getPlan();
}
