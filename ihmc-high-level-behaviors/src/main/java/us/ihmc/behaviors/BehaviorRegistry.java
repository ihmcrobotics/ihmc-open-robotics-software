package us.ihmc.behaviors;

import us.ihmc.behaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.behaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.navigation.NavigationBehavior;
import us.ihmc.behaviors.patrol.PatrolBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

import java.util.HashSet;
import java.util.LinkedHashSet;

/**
 * This class is mostly for using Messager and supporting behaviors defining their
 * Messager APIs in code that depends on this.
 */
public class BehaviorRegistry
{
   public static final BehaviorRegistry DEFAULT_BEHAVIORS = new BehaviorRegistry(LookAndStepBehavior.DEFINITION);
   public static final BehaviorRegistry ARCHIVED_BEHAVIORS = new BehaviorRegistry(ExploreAreaBehavior.DEFINITION);
   static
   {
      DEFAULT_BEHAVIORS.register(LookAndStepBehavior.DEFINITION);
      DEFAULT_BEHAVIORS.register(TraverseStairsBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(StepInPlaceBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(PatrolBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(FancyPosesBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(ExploreAreaBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(NavigationBehavior.DEFINITION);
   }

   private static volatile MessagerAPI MESSAGER_API;
   private static volatile BehaviorRegistry ACTIVE_REGISTRY;

   private final BehaviorDefinition highestLevelNode;
   private final LinkedHashSet<BehaviorDefinition> definitionEntries = new LinkedHashSet<>();

   public static BehaviorRegistry of(BehaviorDefinition highestLevelNode, BehaviorDefinition... entries)
   {
      BehaviorRegistry registry = new BehaviorRegistry(highestLevelNode);
      for (BehaviorDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public BehaviorRegistry(BehaviorDefinition highestLevelNode)
   {
      this.highestLevelNode = highestLevelNode;
   }
   public void register(BehaviorDefinition definition)
   {
      definitionEntries.add(definition);
   }

   public void activateRegistry()
   {
      getMessagerAPI();
   }

   public synchronized MessagerAPI getMessagerAPI()
   {
      if (MESSAGER_API == null) // MessagerAPI can only be created once
      {
         MessagerAPI[] behaviorAPIs = new MessagerAPI[definitionEntries.size()];
         int i = 0;
         for (BehaviorDefinition definitionEntry : definitionEntries)
         {
            behaviorAPIs[i++] = definitionEntry.getBehaviorAPI();
         }
         MESSAGER_API = BehaviorModule.API.create(behaviorAPIs);
         ACTIVE_REGISTRY = this;
      }
      else if (!containsSameSetOfBehaviors(ACTIVE_REGISTRY))
      {
         throw new RuntimeException("Only one set of behaviors can be initialized per process.");
      }

      return MESSAGER_API;
   }

   public LinkedHashSet<BehaviorDefinition> getDefinitionEntries()
   {
      return definitionEntries;
   }

   public boolean containsSameSetOfBehaviors(BehaviorRegistry otherBehaviorRegistry)
   {
      HashSet<BehaviorDefinition> theseBehaviors = new HashSet<>(definitionEntries);
      HashSet<BehaviorDefinition> thoseBehaviors = new HashSet<>(otherBehaviorRegistry.definitionEntries);
      return theseBehaviors.equals(thoseBehaviors);
   }

   public static BehaviorRegistry getActiveRegistry()
   {
      return ACTIVE_REGISTRY;
   }

   public BehaviorDefinition getHighestLevelNode()
   {
      return highestLevelNode;
   }

}
