package us.ihmc.humanoidBehaviors;

import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

import java.util.LinkedHashSet;

public class BehaviorRegistry
{
   public static final BehaviorRegistry DEFAULT_BEHAVIORS = new BehaviorRegistry();
   static
   {
      DEFAULT_BEHAVIORS.register(StepInPlaceBehavior.DEFINITION);
      DEFAULT_BEHAVIORS.register(PatrolBehavior.DEFINITION);
      DEFAULT_BEHAVIORS.register(FancyPosesBehavior.DEFINITION);
      DEFAULT_BEHAVIORS.register(ExploreAreaBehavior.DEFINITION);
   }

   private final LinkedHashSet<BehaviorDefinition> definitionEntries = new LinkedHashSet<>();
   private MessagerAPI messagerAPI;

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
      if (messagerAPI == null) // MessagerAPI can only be created once
      {
         MessagerAPI[] behaviorAPIs = new MessagerAPI[definitionEntries.size()];
         int i = 0;
         for (BehaviorDefinition definitionEntry : definitionEntries)
         {
            behaviorAPIs[i++] = definitionEntry.getBehaviorAPI();
         }
         messagerAPI = BehaviorModule.API.create(behaviorAPIs);
      }
      return messagerAPI;
   }

   public LinkedHashSet<BehaviorDefinition> getDefinitionEntries()
   {
      return definitionEntries;
   }
}
