package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;

public class PerNodeCost implements PawNodeCost
{
   private final PawPlannerParametersReadOnly parameters;

   public PerNodeCost(PawPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      return parameters.getCostPerStep();
   }
}
