package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerHeuristicCalculator;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;

public interface IdealStepCalculatorInterface
{
   DiscreteFootstep computeIdealStep(DiscreteFootstep stanceNode, DiscreteFootstep startOfSwing);

   default FootstepPlannerHeuristicCalculator getFootstepPlannerHeuristicCalculator()
   {
      return null;
   }
}
