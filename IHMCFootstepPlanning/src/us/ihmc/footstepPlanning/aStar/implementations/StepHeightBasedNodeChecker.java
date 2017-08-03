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
      RigidBodyTransform nodeToPreviousNodeTransform = new RigidBodyTransform(node.getSoleTransform());
      nodeToPreviousNodeTransform.transform(previosNode.getSoleTransform());
      double heightChange = Math.abs(nodeToPreviousNodeTransform.getTranslationZ());
      return heightChange < maxStepHeightChange;
   }
}
