package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class EuclideanBasedCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;

   public EuclideanBasedCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
      this.costParameters = parameters.getCostParameters();
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return startNode.euclideanDistance(endNode) + costParameters.getCostPerStep();
   }
}
