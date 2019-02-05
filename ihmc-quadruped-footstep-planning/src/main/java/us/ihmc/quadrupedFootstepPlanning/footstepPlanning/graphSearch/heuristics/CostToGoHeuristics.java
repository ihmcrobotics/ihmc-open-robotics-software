package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class CostToGoHeuristics
{
   protected final FootstepPlannerParameters parameters;

   public CostToGoHeuristics(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   public double getWeight()
   {
      return parameters.getHeuristicsWeight();
   }

   public double compute(FootstepNode node, FootstepNode goalNode)
   {
      return parameters.getHeuristicsWeight() * computeHeuristics(node, goalNode);
   }

   public abstract void setGoalHasBeenReached(boolean bodyHasReachedGoal);

   protected abstract double computeHeuristics(FootstepNode node, FootstepNode goalNode);
}
