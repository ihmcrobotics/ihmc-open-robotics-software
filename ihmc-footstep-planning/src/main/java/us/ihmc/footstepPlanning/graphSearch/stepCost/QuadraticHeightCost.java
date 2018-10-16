package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;

public class QuadraticHeightCost implements FootstepCost
{
   private static final double stepHeightScalar = 10.0;

   private final FootstepPlannerCostParameters costParameters;

   public QuadraticHeightCost(FootstepPlannerCostParameters costParameters)
   {
      this.costParameters = costParameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (!startNode.hasZ() || !endNode.hasZ())
         return 0.0;

      double heightChange = endNode.getZ() - startNode.getZ();

      if (heightChange > 0.0)
         return costParameters.getStepUpWeight() * Math.pow(stepHeightScalar * heightChange, 2.0);
      else
         return costParameters.getStepDownWeight() * Math.pow(stepHeightScalar * heightChange, 2.0);
   }
}
