package us.ihmc.gdx.ui.behaviors.registry;

import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.gdx.ui.behaviors.ImGuiGDXBuildingExplorationBehaviorUI;
import us.ihmc.gdx.ui.behaviors.ImGuiGDXDoorBehaviorUI;
import us.ihmc.gdx.ui.behaviors.ImGuiGDXLookAndStepBehaviorUI;
import us.ihmc.gdx.ui.behaviors.ImGuiGDXTraverseStairsBehaviorUI;

import java.util.LinkedHashSet;

public class GDXBehaviorUIRegistry extends BehaviorRegistry
{
   public static final GDXBehaviorUIRegistry DEFAULT_BEHAVIORS = new GDXBehaviorUIRegistry(ImGuiGDXBuildingExplorationBehaviorUI.DEFINITION);
   static
   {
      DEFAULT_BEHAVIORS.register(ImGuiGDXLookAndStepBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXBuildingExplorationBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXDoorBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(ImGuiGDXTraverseStairsBehaviorUI.DEFINITION);
   }

   private final LinkedHashSet<GDXBehaviorUIDefinition> uiDefinitionEntries = new LinkedHashSet<>();
   private int numberOfUIs = 0;
   private String nameOfOnlyUIBehavior;

   public static GDXBehaviorUIRegistry of(GDXBehaviorUIDefinition highestLevelNode, GDXBehaviorUIDefinition... entries)
   {
      GDXBehaviorUIRegistry registry = new GDXBehaviorUIRegistry(highestLevelNode);
      for (GDXBehaviorUIDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public GDXBehaviorUIRegistry(GDXBehaviorUIDefinition highestLevelNode)
   {
      super(highestLevelNode);
   }

   public void register(GDXBehaviorUIDefinition definition)
   {
      super.register(definition);
      uiDefinitionEntries.add(definition);

      if (definition.getBehaviorUISupplier() != null)
      {
         ++numberOfUIs;
         nameOfOnlyUIBehavior = definition.getName();
      }
   }

   public LinkedHashSet<GDXBehaviorUIDefinition> getUIDefinitionEntries()
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
   public GDXBehaviorUIDefinition getHighestLevelNode()
   {
      return (GDXBehaviorUIDefinition) super.getHighestLevelNode();
   }
}
