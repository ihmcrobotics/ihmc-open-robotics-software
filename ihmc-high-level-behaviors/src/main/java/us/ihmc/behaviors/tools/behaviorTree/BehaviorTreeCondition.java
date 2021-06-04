package us.ihmc.behaviors.tools.behaviorTree;

import java.util.function.BooleanSupplier;

/**
 * A behavior tree action that draws from a boolean supplier.
 */
public class BehaviorTreeCondition extends BehaviorTreeAction implements BehaviorTreeConditionBasics
{
   private final BooleanSupplier conditionSupplier;

   public BehaviorTreeCondition(BooleanSupplier conditionSupplier)
   {
      this.conditionSupplier = conditionSupplier;
      setType(BehaviorTreeCondition.class);
   }

   @Override
   public boolean checkCondition()
   {
      return conditionSupplier.getAsBoolean();
   }
}
