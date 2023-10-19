package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;

/**
 * Definitions are not freezable.
 */
public class BehaviorTreeNodeDefinitionAddition implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeDefinition nodeToAdd;
   private final BehaviorTreeNodeDefinition parent;

   public BehaviorTreeNodeDefinitionAddition(BehaviorTreeNodeDefinition nodeToAdd, BehaviorTreeNodeDefinition parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
   }

   protected BehaviorTreeNodeDefinition getNodeToAdd()
   {
      return nodeToAdd;
   }

   protected BehaviorTreeNodeDefinition getParent()
   {
      return parent;
   }
}
