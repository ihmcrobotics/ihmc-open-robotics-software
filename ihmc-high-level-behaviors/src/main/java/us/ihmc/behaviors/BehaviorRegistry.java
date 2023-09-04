package us.ihmc.behaviors;

import us.ihmc.behaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.behaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.navigation.NavigationBehavior;
import us.ihmc.behaviors.patrol.PatrolBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;

import java.util.LinkedHashSet;

/**
 * FIXME: This class was designed for using Messager and supporting behaviors defining their
 *  Messager APIs in code that depends on this. We have removed Messager, so the design needs
 *  to be reconsidered.
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

   private BehaviorDefinition highestLevelNode;
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

   public LinkedHashSet<BehaviorDefinition> getDefinitionEntries()
   {
      return definitionEntries;
   }

   public BehaviorDefinition getHighestLevelNode()
   {
      return highestLevelNode;
   }

   public void setHighestLevelNode(BehaviorDefinition highestLevelNode)
   {
      this.highestLevelNode = highestLevelNode;
   }

   public BehaviorDefinition getBehaviorFromName(String behaviorName)
   {
      for (BehaviorDefinition definitionEntry : definitionEntries)
      {
         if (definitionEntry.getName().equals(behaviorName))
         {
            return definitionEntry;
         }
      }
      return null;
   }
}
