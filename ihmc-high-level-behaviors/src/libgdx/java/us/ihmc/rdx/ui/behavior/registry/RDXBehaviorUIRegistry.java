package us.ihmc.rdx.ui.behavior.registry;

import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.rdx.ui.behavior.behaviors.*;

import java.util.Comparator;
import java.util.TreeSet;

public class RDXBehaviorUIRegistry extends BehaviorRegistry
{
   public static final RDXBehaviorUIRegistry DEFAULT_BEHAVIORS = new RDXBehaviorUIRegistry(RDXLookAndStepBehaviorUI.DEFINITION);
   static
   {
      DEFAULT_BEHAVIORS.register(RDXTargetFollowingBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(RDXLookAndStepBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(RDXBuildingExplorationBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(RDXDoorBehaviorUI.DEFINITION);
      DEFAULT_BEHAVIORS.register(RDXTraverseStairsBehaviorUI.DEFINITION);
   }

   private final TreeSet<RDXBehaviorUIDefinition> uiDefinitionEntries = new TreeSet<>(Comparator.comparing(RDXBehaviorUIDefinition::getName));

   public static RDXBehaviorUIRegistry of(RDXBehaviorUIDefinition highestLevelNode, RDXBehaviorUIDefinition... entries)
   {
      RDXBehaviorUIRegistry registry = new RDXBehaviorUIRegistry(highestLevelNode);
      for (RDXBehaviorUIDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public RDXBehaviorUIRegistry(RDXBehaviorUIDefinition highestLevelNode)
   {
      super(highestLevelNode);
   }

   public void register(RDXBehaviorUIDefinition definition)
   {
      super.register(definition);
      uiDefinitionEntries.add(definition);
   }

   public TreeSet<RDXBehaviorUIDefinition> getUIDefinitionEntries()
   {
      return uiDefinitionEntries;
   }

   @Override
   public RDXBehaviorUIDefinition getHighestLevelNode()
   {
      return (RDXBehaviorUIDefinition) super.getHighestLevelNode();
   }
}
