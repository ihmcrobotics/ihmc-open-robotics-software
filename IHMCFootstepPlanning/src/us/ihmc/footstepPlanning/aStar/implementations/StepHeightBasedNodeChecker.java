package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeChecker;

public class StepHeightBasedNodeChecker implements FootstepNodeChecker
{
   private static final double maxStepHeightChange = 0.2;

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode)
   {
      double nodeHeight = node.getSoleTransform().getTranslationZ();
      double previousNodeHeight = previosNode.getSoleTransform().getTranslationZ();
      double heightChange = Math.abs(nodeHeight - previousNodeHeight);
      return heightChange < maxStepHeightChange;
   }
}
