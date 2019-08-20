package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;

public class PawHeightCost implements PawNodeCost
{
   private final PawLinearHeightCost linearHeightCost;

   public PawHeightCost(PawPlannerParameters parameters, PawNodeSnapperReadOnly snapper)
   {
      linearHeightCost = new PawLinearHeightCost(parameters, snapper);
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      return linearHeightCost.compute(startNode, endNode);
   }
}
