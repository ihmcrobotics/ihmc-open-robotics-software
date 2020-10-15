package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;

public interface IdealStepCalculatorInterface
{
   DiscreteFootstep computeIdealStep(DiscreteFootstep stanceNode, DiscreteFootstep startOfSwing);
}
