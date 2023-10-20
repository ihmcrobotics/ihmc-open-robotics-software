package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

/**
 * Adds a node and freezes the parent.
 */
public class BehaviorTreeStateNodeAddition implements BehaviorTreeStateModification
{
   private final BehaviorTreeDefinitionNodeAddition definitionAddition;

   private final BehaviorTreeNodeState nodeToAdd;
   private final BehaviorTreeNodeState parent;

   public BehaviorTreeStateNodeAddition(BehaviorTreeNodeState nodeToAdd, BehaviorTreeNodeState parent)
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
      parent.freezeFromModification();
   }
}
