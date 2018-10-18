package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class PerStepCost implements FootstepCost
{
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;

   public PerStepCost(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
      this.costParameters = parameters.getCostParameters();
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return costParameters.getCostPerStep();
   }
}
