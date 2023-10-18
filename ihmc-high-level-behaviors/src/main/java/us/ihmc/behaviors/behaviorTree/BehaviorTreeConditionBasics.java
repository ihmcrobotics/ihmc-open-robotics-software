package us.ihmc.behaviors.behaviorTree;

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
         return BehaviorTreeNodeStatus.SUCCESS;
      }
      else
      {
         return BehaviorTreeNodeStatus.FAILURE;
      }
   }
}
