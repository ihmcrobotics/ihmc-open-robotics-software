package us.ihmc.gdx.ui.behavior.registry;

import us.ihmc.behaviors.BehaviorDefinition;

public class ImGuiGDXBehaviorUIDefinition extends BehaviorDefinition
{
   private final ImGuiGDXBehaviorUIInterfaceConstructor behaviorUISupplier;

   public ImGuiGDXBehaviorUIDefinition(BehaviorDefinition definition, ImGuiGDXBehaviorUIInterfaceConstructor behaviorUISupplier)
   {
      super(definition.getName(),
            definition.getBehaviorSupplier(),
            definition.getBehaviorAPI(),
            definition.getSubBehaviors().toArray(new BehaviorDefinition[0]));
      this.behaviorUISupplier = behaviorUISupplier;
   }

   public ImGuiGDXBehaviorUIInterfaceConstructor getBehaviorUISupplier()
   {
      return behaviorUISupplier;
   }
}
