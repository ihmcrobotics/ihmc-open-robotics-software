package us.ihmc.humanoidBehaviors;

import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.navigation.NavigationBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

import java.util.HashSet;
import java.util.LinkedHashSet;

public class BehaviorRegistry
{
   public static final BehaviorRegistry DEFAULT_BEHAVIORS = new BehaviorRegistry();
   public static final BehaviorRegistry ARCHIVED_BEHAVIORS = new BehaviorRegistry();
   static
   {
      DEFAULT_BEHAVIORS.register(LookAndStepBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(StepInPlaceBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(PatrolBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(FancyPosesBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(ExploreAreaBehavior.DEFINITION);
      ARCHIVED_BEHAVIORS.register(NavigationBehavior.DEFINITION);
   }

   private static volatile MessagerAPI MESSAGER_API;
   private static volatile BehaviorRegistry ACTIVE_REGISTRY;

   private final LinkedHashSet<BehaviorDefinition> definitionEntries = new LinkedHashSet<>();

   public static BehaviorRegistry of(BehaviorDefinition... entries)
   {
      BehaviorRegistry registry = new BehaviorRegistry();
      for (BehaviorDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public void register(BehaviorDefinition definition)
   {
      definitionEntries.add(definition);
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
}
