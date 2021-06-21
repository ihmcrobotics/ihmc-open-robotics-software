package us.ihmc.behaviors.tools.behaviorTree;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.FAILURE;
import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

/**
 * A behavior tree action that draws from a boolean supplier.
 */
public interface BehaviorTreeConditionBasics extends BehaviorTreeActionBasics
{
   public abstract boolean checkCondition();

   @Override
   public default BehaviorTreeNodeStatus tickInternal()
   {
      boolean success = checkCondition();

      if (success)
      {
         return SUCCESS;
      }
      else
      {
         return FAILURE;
      }
   }
}
