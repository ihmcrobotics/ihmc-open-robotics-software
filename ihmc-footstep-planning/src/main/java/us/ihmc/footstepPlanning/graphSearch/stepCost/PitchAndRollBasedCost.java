package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;

public class PitchAndRollBasedCost implements FootstepCost
{
   private final FootstepPlannerCostParameters costParameters;

   public PitchAndRollBasedCost(FootstepPlannerCostParameters costParameters)
   {
      this.costParameters = costParameters;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (!endNode.hasPitch() || !endNode.hasRoll())
         return 0.0;

      return costParameters.getPitchWeight() * Math.abs(endNode.getPitch()) + costParameters.getRollWeight() * Math.abs(endNode.getRoll());
   }
}
