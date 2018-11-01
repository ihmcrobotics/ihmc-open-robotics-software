package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This class defines a listener for new planner goal recommendations. It is typically attached to the
 * {@link PlannerGoalAdditionActionPolicy}, so that when a new planner goal is defined, the listener is notified.
 */
public interface PlannerGoalRecommendationListener extends PlannerHeuristicActionListener
{
   void notifyWithPlannerGoalRecommendation(FootstepPlannerGoal footstepPlannerGoal, RobotSide lastStepSide);
}
