package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.ArrayList;
import java.util.List;

public class CompositeFootstepCost implements FootstepCost
{
   private final List<FootstepCost> footstepCosts;

   public CompositeFootstepCost()
   {
      this.footstepCosts = new ArrayList<>();
   }

   public CompositeFootstepCost(List<FootstepCost> footstepCosts)
   {
      this.footstepCosts = footstepCosts;
   }

   public void addFootstepCost(FootstepCost footstepCost)
   {
      footstepCosts.add(footstepCost);
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      double cost = 0.0;
      for (FootstepCost footstepCost : footstepCosts)
         cost += footstepCost.compute(startNode, endNode);

      return cost;
   }
}
