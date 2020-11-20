package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.humanoidBehaviors.BehaviorDefinition;

public class BehaviorUIDefinition extends BehaviorDefinition
{
   private final BehaviorUIInterfaceConstructor behaviorUISupplier;

   public BehaviorUIDefinition(BehaviorDefinition definition, BehaviorUIInterfaceConstructor behaviorUISupplier)
   {
      super(definition.getName(), definition.getBehaviorSupplier(), definition.getBehaviorAPI());
      this.behaviorUISupplier = behaviorUISupplier;
   }

   public BehaviorUIInterfaceConstructor getBehaviorUISupplier()
   {
      return behaviorUISupplier;
   }
}
