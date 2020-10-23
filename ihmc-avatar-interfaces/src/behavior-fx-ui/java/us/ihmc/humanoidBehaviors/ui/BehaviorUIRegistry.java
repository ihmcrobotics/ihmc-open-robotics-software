package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.*;

import java.util.LinkedHashSet;

public class BehaviorUIRegistry extends BehaviorRegistry
{
   public static final BehaviorUIRegistry DEFAULT_BEHAVIORS = new BehaviorUIRegistry();
   public static final BehaviorUIRegistry ARCHIVED_BEHAVIORS = new BehaviorUIRegistry();
   static
   {
      DEFAULT_BEHAVIORS.register(LookAndStepBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(StepInPlaceBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(PatrolBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(FancyPosesBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(ExploreAreaBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(NavigationBehaviorUI.DEFINITION);
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
