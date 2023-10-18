package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;

/**
 * There is no need for destroying definitions, so we let the GC get them.
 */
public class BehaviorTreeDefinitionClearSubtree implements BehaviorTreeDefinitionModification
{
   private final BehaviorTreeNodeDefinition subtreeToClear;

   public BehaviorTreeDefinitionClearSubtree(BehaviorTreeNodeDefinition subtreeToClear)
   {
      this.subtreeToClear = subtreeToClear;
   }

   @Override
   public void performOperation()
   {
      clearChildren(subtreeToClear);
   }

   private void clearChildren(BehaviorTreeNodeDefinition localNode)
   {
      for (BehaviorTreeNodeDefinition child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getChildren().clear();
   }
}

