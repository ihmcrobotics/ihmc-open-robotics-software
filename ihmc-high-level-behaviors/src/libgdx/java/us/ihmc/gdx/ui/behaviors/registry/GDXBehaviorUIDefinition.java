package us.ihmc.gdx.ui.behaviors.registry;

import us.ihmc.humanoidBehaviors.BehaviorDefinition;

public class GDXBehaviorUIDefinition extends BehaviorDefinition
{
   private final GDXBehaviorUIInterfaceConstructor behaviorUISupplier;

   public GDXBehaviorUIDefinition(BehaviorDefinition definition, GDXBehaviorUIInterfaceConstructor behaviorUISupplier)
   {
      super(definition.getName(),
            definition.getBehaviorSupplier(),
            definition.getBehaviorAPI(),
            definition.getSubBehaviors().toArray(new BehaviorDefinition[0]));
      this.behaviorUISupplier = behaviorUISupplier;
   }

   public GDXBehaviorUIInterfaceConstructor getBehaviorUISupplier()
   {
      return behaviorUISupplier;
   }
}
