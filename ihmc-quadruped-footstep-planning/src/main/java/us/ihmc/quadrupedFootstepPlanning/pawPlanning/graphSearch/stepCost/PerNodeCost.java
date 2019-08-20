package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;

public class PerNodeCost implements PawNodeCost
{
   private final PawPlannerParameters parameters;

   public PerNodeCost(PawPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      return parameters.getCostPerStep();
   }
}
