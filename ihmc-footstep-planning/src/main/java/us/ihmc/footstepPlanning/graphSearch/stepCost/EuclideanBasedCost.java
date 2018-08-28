package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class EuclideanBasedCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;

   public EuclideanBasedCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return startNode.euclideanDistance(endNode) + parameters.getCostPerStep();
   }
}
