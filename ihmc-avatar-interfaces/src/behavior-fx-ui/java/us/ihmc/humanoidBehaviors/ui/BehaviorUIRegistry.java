package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.ExploreAreaBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.behaviors.FancyPosesBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.behaviors.PatrolBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.behaviors.StepInPlaceBehaviorUI;

import java.util.LinkedHashSet;

public class BehaviorUIRegistry extends BehaviorRegistry
{
   public static final BehaviorUIRegistry DEFAULT_BEHAVIORS = new BehaviorUIRegistry();
   static
   {
      DEFAULT_BEHAVIORS.register(StepInPlaceBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(PatrolBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(FancyPosesBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ExploreAreaBehaviorUI.DEFINITION);
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
