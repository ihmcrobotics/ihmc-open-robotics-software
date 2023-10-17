package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.behaviorTree.utility.UtilityBasedAction;

public class UtilitySelectorNode extends UtilityBasedAction
{
   public UtilitySelectorNode()
   {

   }

   @Override
   protected BehaviorTreeNodeStatus tickInternal()
   {
      double highestUtility = 0.0;
      BehaviorTreeNodeExecutor nodeOfHighestUtility = null;
      for (BehaviorTreeNodeExecutor child : getChildren())
      {
         double utility = evaluateUtility();
         if (utility > highestUtility)
         {
            highestUtility = utility;
            nodeOfHighestUtility = child;
         }
      }

      if (highestUtility > 0.0)
      {
         return nodeOfHighestUtility.tick();
      }
      else
      {
         return BehaviorTreeNodeStatus.FAILURE;
      }
   }
}
