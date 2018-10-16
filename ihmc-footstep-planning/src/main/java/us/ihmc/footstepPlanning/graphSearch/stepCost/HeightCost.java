package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;

public class HeightCost implements FootstepCost
{
   private final FootstepPlannerCostParameters costParameters;

   private final LinearHeightCost linearHeightCost;
   private final QuadraticHeightCost quadraticHeightCost;

   public HeightCost(FootstepPlannerCostParameters costParameters)
   {
      this.costParameters = costParameters;

      linearHeightCost = new LinearHeightCost(costParameters);
      quadraticHeightCost = new QuadraticHeightCost(costParameters);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (costParameters.useQuadraticHeightCost())
         return quadraticHeightCost.compute(startNode, endNode);
      else
         return linearHeightCost.compute(startNode, endNode);
   }
}
