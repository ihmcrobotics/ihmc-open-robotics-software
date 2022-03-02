package us.ihmc.gdx.ui.behavior.registry;

import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.gdx.ui.behavior.behaviors.*;

import java.util.Comparator;
import java.util.TreeSet;

public class ImGuiGDXBehaviorUIRegistry extends BehaviorRegistry
{
   public static final ImGuiGDXBehaviorUIRegistry DEFAULT_BEHAVIORS = new ImGuiGDXBehaviorUIRegistry(ImGuiGDXLookAndStepBehaviorUI.DEFINITION);
   static
   {
      DEFAULT_BEHAVIORS.register(ImGuiGDXTargetFollowingBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXLookAndStepBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXBuildingExplorationBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXDoorBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXTraverseStairsBehaviorUI.DEFINITION);
   }

   private final TreeSet<ImGuiGDXBehaviorUIDefinition> uiDefinitionEntries = new TreeSet<>(Comparator.comparing(ImGuiGDXBehaviorUIDefinition::getName));

   public static ImGuiGDXBehaviorUIRegistry of(ImGuiGDXBehaviorUIDefinition highestLevelNode, ImGuiGDXBehaviorUIDefinition... entries)
   {
      ImGuiGDXBehaviorUIRegistry registry = new ImGuiGDXBehaviorUIRegistry(highestLevelNode);
      for (ImGuiGDXBehaviorUIDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public ImGuiGDXBehaviorUIRegistry(ImGuiGDXBehaviorUIDefinition highestLevelNode)
   {
      super(highestLevelNode);
   }

   public void register(ImGuiGDXBehaviorUIDefinition definition)
   {
      super.register(definition);
      uiDefinitionEntries.add(definition);
   }

   public TreeSet<ImGuiGDXBehaviorUIDefinition> getUIDefinitionEntries()
   {
      return uiDefinitionEntries;
   }

   @Override
   public ImGuiGDXBehaviorUIDefinition getHighestLevelNode()
   {
      return (ImGuiGDXBehaviorUIDefinition) super.getHighestLevelNode();
   }
}
