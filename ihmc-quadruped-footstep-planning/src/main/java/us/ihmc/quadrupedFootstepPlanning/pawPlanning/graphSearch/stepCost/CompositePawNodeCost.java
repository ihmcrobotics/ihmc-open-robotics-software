package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;

import java.util.ArrayList;
import java.util.List;

public class CompositePawNodeCost implements PawNodeCost
{
   private final List<PawNodeCost> pawNodeCosts;

   public CompositePawNodeCost()
   {
      this.pawNodeCosts = new ArrayList<>();
   }

   public CompositePawNodeCost(List<PawNodeCost> pawNodeCosts)
   {
      this.pawNodeCosts = pawNodeCosts;
   }

   public void addPawNodeCost(PawNodeCost pawNodeCost)
   {
      pawNodeCosts.add(pawNodeCost);
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      double cost = 0.0;
      for (PawNodeCost pawNodeCost : pawNodeCosts)
         cost += pawNodeCost.compute(startNode, endNode);

      return cost;
   }
}
