package us.ihmc.behaviors.javafx;

import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.javafx.behaviors.*;
import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.javafx.behaviors.coordinator.BuildingExplorationBehaviorUI;

import java.util.LinkedHashSet;

public class JavaFXBehaviorUIRegistry extends BehaviorRegistry
{
   public static final JavaFXBehaviorUIRegistry DEFAULT_BEHAVIORS = new JavaFXBehaviorUIRegistry(BuildingExplorationBehaviorUI.DEFINITION);
   public static final JavaFXBehaviorUIRegistry ARCHIVED_BEHAVIORS = new JavaFXBehaviorUIRegistry(ExploreAreaBehaviorUI.DEFINITION);
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

   private final LinkedHashSet<JavaFXBehaviorUIDefinition> uiDefinitionEntries = new LinkedHashSet<>();
   private int numberOfUIs = 0;
   private String nameOfOnlyUIBehavior;

   public static JavaFXBehaviorUIRegistry of(BehaviorDefinition highestLevelNode, JavaFXBehaviorUIDefinition... entries)
   {
      JavaFXBehaviorUIRegistry registry = new JavaFXBehaviorUIRegistry(highestLevelNode);
      for (JavaFXBehaviorUIDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public JavaFXBehaviorUIRegistry(BehaviorDefinition highestLevelNode)
   {
      super(highestLevelNode);
   }

   public void register(JavaFXBehaviorUIDefinition definition)
   {
      super.register(definition);
      uiDefinitionEntries.add(definition);

      if (definition.getBehaviorUISupplier() != null)
      {
         ++numberOfUIs;
         nameOfOnlyUIBehavior = definition.getName();
      }
   }

   public LinkedHashSet<JavaFXBehaviorUIDefinition> getUIDefinitionEntries()
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
