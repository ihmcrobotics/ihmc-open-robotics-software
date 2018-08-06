package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class PitchAndRollBasedCost implements FootstepCost
{
   private static final double pitchCost = 0.5;
   private static final double rollCost = 2.0;

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      if (!endNode.hasPitch() || !endNode.hasRoll())
         return 0.0;

      return pitchCost * Math.abs(endNode.getPitch()) + rollCost * Math.abs(endNode.getRoll());
   }
}
