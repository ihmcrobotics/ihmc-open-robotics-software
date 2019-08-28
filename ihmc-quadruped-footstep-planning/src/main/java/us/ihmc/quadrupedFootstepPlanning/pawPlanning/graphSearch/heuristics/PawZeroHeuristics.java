package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;

public class PawZeroHeuristics extends PawPlanningCostToGoHeuristics
{
   public PawZeroHeuristics(PawStepPlannerParametersReadOnly parameters)
   {
      super(parameters);
   }

   @Override
   protected double computeHeuristics(PawNode node, PawNode goalNode)
   {
      return 0.0;
   }
}
