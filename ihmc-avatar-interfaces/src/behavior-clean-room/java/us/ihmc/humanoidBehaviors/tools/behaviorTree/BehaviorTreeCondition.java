package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import java.util.function.BooleanSupplier;

public class BehaviorTreeCondition implements BehaviorTreeNode
{
   private final BooleanSupplier conditionSupplier;

   public BehaviorTreeCondition(BooleanSupplier conditionSupplier)
   {
      this.conditionSupplier = conditionSupplier;
   }

   protected boolean checkCondition()
   {
      return conditionSupplier.getAsBoolean();
   }

   @Override
   public BehaviorTreeNodeStatus tick()
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
