package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

public class PerStepCost implements FootstepCost
{
   private final FootstepPlannerParametersReadOnly parameters;

   public PerStepCost(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return parameters.getCostPerStep();
   }
}
