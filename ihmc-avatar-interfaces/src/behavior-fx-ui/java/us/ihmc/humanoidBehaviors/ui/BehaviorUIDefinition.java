package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.humanoidBehaviors.BehaviorDefinition;

import java.util.function.Supplier;

public class BehaviorUIDefinition extends BehaviorDefinition
{
   private final Supplier<BehaviorUIInterface> behaviorUISupplier;

   public BehaviorUIDefinition(BehaviorDefinition definition, Supplier<BehaviorUIInterface> behaviorUISupplier)
   {
      super(definition.getName(), definition.getBehaviorSupplier(), definition.getBehaviorAPI());
      this.behaviorUISupplier = behaviorUISupplier;
   }

   public Supplier<BehaviorUIInterface> getBehaviorUISupplier()
   {
      return behaviorUISupplier;
   }
}
