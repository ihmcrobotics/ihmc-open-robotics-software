package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.behaviorTree.utility.UtilityBasedAction;

public class UtilitySelectorNode extends UtilityBasedAction
{
   public UtilitySelectorNode()
   {

   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      double highestUtility = 0.0;
      UtilityBasedAction nodeOfHighestUtility = null;
      for (UtilityBasedAction child : getUtilityChildren())
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
         return nodeOfHighestUtility.tickAndGetStatus();
      }
      else
      {
         return BehaviorTreeNodeStatus.FAILURE;
      }
   }
}
