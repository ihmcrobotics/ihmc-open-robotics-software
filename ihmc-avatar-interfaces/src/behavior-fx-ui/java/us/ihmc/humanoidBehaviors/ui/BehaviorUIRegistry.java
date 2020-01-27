package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.ExploreAreaBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.FancyPosesBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.PatrolBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.StepInPlaceBehaviorUIController;

import java.util.LinkedHashSet;

public class BehaviorUIRegistry extends BehaviorRegistry
{
   public static final BehaviorUIRegistry DEFAULT_BEHAVIORS = new BehaviorUIRegistry();
   static
   {
      DEFAULT_BEHAVIORS.register(StepInPlaceBehaviorUIController.DEFINITION);
      DEFAULT_BEHAVIORS.register(PatrolBehaviorUIController.DEFINITION);
      DEFAULT_BEHAVIORS.register(FancyPosesBehaviorUIController.DEFINITION);
      DEFAULT_BEHAVIORS.register(ExploreAreaBehaviorUIController.DEFINITION);
   }

   private final LinkedHashSet<BehaviorUIDefinition> uiDefinitionEntries = new LinkedHashSet<>();

   public static BehaviorUIRegistry of(BehaviorUIDefinition... entries)
   {
      BehaviorUIRegistry registry = new BehaviorUIRegistry();
      for (BehaviorUIDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public void register(BehaviorUIDefinition definition)
   {
      super.register(definition);
      uiDefinitionEntries.add(definition);
   }

   public LinkedHashSet<BehaviorUIDefinition> getUIDefinitionEntries()
   {
      return uiDefinitionEntries;
   }
}
