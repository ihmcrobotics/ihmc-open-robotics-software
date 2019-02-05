package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;

public class HeightCost implements FootstepCost
{
   private final LinearHeightCost linearHeightCost;

   public HeightCost(FootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapper)
   {
      linearHeightCost = new LinearHeightCost(parameters, snapper);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      return linearHeightCost.compute(startNode, endNode);
   }
}
