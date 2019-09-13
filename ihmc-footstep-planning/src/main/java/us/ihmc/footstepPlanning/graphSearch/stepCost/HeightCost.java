package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

public class HeightCost implements FootstepCost
{
   private final FootstepPlannerParametersReadOnly parameters;

   private final LinearHeightCost linearHeightCost;
   private final QuadraticHeightCost quadraticHeightCost;

   public HeightCost(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;

      linearHeightCost = new LinearHeightCost(parameters, snapper);
      quadraticHeightCost = new QuadraticHeightCost(parameters, snapper);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (parameters.useQuadraticHeightCost())
         return quadraticHeightCost.compute(startNode, endNode);
      else
         return linearHeightCost.compute(startNode, endNode);
   }
}
