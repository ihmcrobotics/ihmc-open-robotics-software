package us.ihmc.rdx.ui.behavior.registry;

import us.ihmc.behaviors.BehaviorDefinition;

public class RDXBehaviorUIDefinition extends BehaviorDefinition
{
   private final RDXBehaviorUIInterfaceConstructor behaviorUISupplier;

   public RDXBehaviorUIDefinition(BehaviorDefinition definition, RDXBehaviorUIInterfaceConstructor behaviorUISupplier)
   {
      super(definition.getName(), definition.getBehaviorSupplier(), definition.getSubBehaviors().toArray(new BehaviorDefinition[0]));
      this.behaviorUISupplier = behaviorUISupplier;
   }

   public RDXBehaviorUIInterfaceConstructor getBehaviorUISupplier()
   {
      return behaviorUISupplier;
   }
}
