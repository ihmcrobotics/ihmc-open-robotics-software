package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.humanoidBehaviors.BehaviorDefinition;

public class BehaviorUIDefinition extends BehaviorDefinition
{
   private final BehaviorUIInterfaceConstructor behaviorUISupplier;

   public BehaviorUIDefinition(BehaviorDefinition definition, BehaviorUIInterfaceConstructor behaviorUISupplier)
   {
      super(definition.getName(),
            definition.getBehaviorSupplier(),
            definition.getBehaviorAPI(),
            definition.getSubBehaviors().toArray(new BehaviorDefinition[0]));
      this.behaviorUISupplier = behaviorUISupplier;
   }

   public BehaviorUIInterfaceConstructor getBehaviorUISupplier()
   {
      return behaviorUISupplier;
   }
}
