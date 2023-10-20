package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

/**
 * Adds a node without freezing the parent.
 */
public class BehaviorTreeStateNodeReplacement implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeState nodeToAdd;
   private final BehaviorTreeNodeState parent;

   private final BehaviorTreeDefinitionNodeAddition definitionAddition;

   public BehaviorTreeStateNodeReplacement(BehaviorTreeNodeState nodeToAdd, BehaviorTreeNodeState parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;

      definitionAddition = new BehaviorTreeDefinitionNodeAddition(nodeToAdd.getDefinition(), parent.getDefinition());
   }

   @Override
   public void performOperation()
   {
      definitionAddition.performOperation();

      parent.getChildren().add(nodeToAdd);
   }
}
