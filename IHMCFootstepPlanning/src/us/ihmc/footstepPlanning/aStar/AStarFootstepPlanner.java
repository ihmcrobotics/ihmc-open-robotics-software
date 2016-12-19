package us.ihmc.footstepPlanning.aStar;

import java.util.PriorityQueue;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

public class AStarFootstepPlanner implements FootstepPlanner
{
   private PriorityQueue<FootstepNode> stack;

   public AStarFootstepPlanner()
   {
      stack = new PriorityQueue<>();
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public FootstepPlanningResult plan()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FootstepPlan getPlan()
   {
      // TODO Auto-generated method stub
      return null;
   }

}
