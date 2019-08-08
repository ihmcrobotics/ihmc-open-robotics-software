package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

public class EuclideanBasedCost implements FootstepCost
{
   private final FootstepPlannerParametersReadOnly parameters;

   public EuclideanBasedCost(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return startNode.euclideanDistance(endNode) + parameters.getCostPerStep();
   }
}
