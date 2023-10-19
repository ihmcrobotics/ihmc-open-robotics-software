package us.ihmc.behaviors;

import java.util.LinkedHashSet;

/**
 * The registry exists to enumerate available root behavior tree nodes.
 * This is needed because they are programmatically defined instead of by
 * being a directory of JSON files.
 */
public class BehaviorRegistry
{
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
