package us.ihmc.behaviors.behaviorTree;

import java.util.function.BooleanSupplier;

/**
 * A behavior tree action that draws from a boolean supplier.
 */
public abstract class BehaviorTreeCondition extends LocalOnlyBehaviorTreeNodeExecutor
{
   private final BooleanSupplier conditionSupplier;

   public BehaviorTreeNodeStatus determineStatus()
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

   public BehaviorTreeCondition(BooleanSupplier conditionSupplier)
   {
      this.conditionSupplier = conditionSupplier;
   }

   public boolean checkCondition()
   {
      return conditionSupplier.getAsBoolean();
   }
}
