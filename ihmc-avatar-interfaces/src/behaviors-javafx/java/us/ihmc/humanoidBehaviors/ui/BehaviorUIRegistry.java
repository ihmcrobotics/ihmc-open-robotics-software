package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehavior;
import us.ihmc.humanoidBehaviors.ui.behaviors.*;
import us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorUI;

import java.util.LinkedHashSet;

public class BehaviorUIRegistry extends BehaviorRegistry
{
   public static final BehaviorUIRegistry DEFAULT_BEHAVIORS = new BehaviorUIRegistry();
   public static final BehaviorUIRegistry ARCHIVED_BEHAVIORS = new BehaviorUIRegistry();
   static
   {
      DEFAULT_BEHAVIORS.register(BuildingExplorationBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ExploreAreaBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(LookAndStepBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(TraverseStairsBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(StepInPlaceBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(PatrolBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(FancyPosesBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(ExploreAreaBehaviorUI.DEFINITION);
      ARCHIVED_BEHAVIORS.register(NavigationBehaviorUI.DEFINITION);
   }

   private final LinkedHashSet<BehaviorUIDefinition> uiDefinitionEntries = new LinkedHashSet<>();
   private int numberOfUIs = 0;
   private String nameOfOnlyUIBehavior;

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

      if (definition.getBehaviorUISupplier() != null)
      {
         ++numberOfUIs;
         nameOfOnlyUIBehavior = definition.getName();
      }
   }

   public LinkedHashSet<BehaviorUIDefinition> getUIDefinitionEntries()
   {
      return uiDefinitionEntries;
   }

   public int getNumberOfUIs()
   {
      return numberOfUIs;
   }

   public String getNameOfOnlyUIBehavior()
   {
      return nameOfOnlyUIBehavior;
   }
}
