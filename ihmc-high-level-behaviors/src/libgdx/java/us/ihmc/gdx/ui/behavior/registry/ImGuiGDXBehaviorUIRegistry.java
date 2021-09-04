package us.ihmc.gdx.ui.behavior.registry;

import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.gdx.ui.behavior.behaviors.*;

import java.util.LinkedHashSet;

public class ImGuiGDXBehaviorUIRegistry extends BehaviorRegistry
{
   public static final ImGuiGDXBehaviorUIRegistry DEFAULT_BEHAVIORS = new ImGuiGDXBehaviorUIRegistry(ImGuiGDXTargetFollowingBehaviorUI.DEFINITION);
   static
   {
      DEFAULT_BEHAVIORS.register(ImGuiGDXTargetFollowingBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXLookAndStepBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXBuildingExplorationBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXDoorBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXTraverseStairsBehaviorUI.DEFINITION);
   }

   private final LinkedHashSet<ImGuiGDXBehaviorUIDefinition> uiDefinitionEntries = new LinkedHashSet<>();
   private int numberOfUIs = 0;
   private String nameOfOnlyUIBehavior;

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

      if (definition.getBehaviorUISupplier() != null)
      {
         ++numberOfUIs;
         nameOfOnlyUIBehavior = definition.getName();
      }
   }

   public LinkedHashSet<ImGuiGDXBehaviorUIDefinition> getUIDefinitionEntries()
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

   @Override
   public ImGuiGDXBehaviorUIDefinition getHighestLevelNode()
   {
      return (ImGuiGDXBehaviorUIDefinition) super.getHighestLevelNode();
   }
}
