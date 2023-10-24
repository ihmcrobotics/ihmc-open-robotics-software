package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionAddAndFreeze<T extends BehaviorTreeNodeExtension<T, ?, ?, ?>>
      extends BehaviorTreeNodeAddAndFreeze<T>
      implements BehaviorTreeModification
{
   private final BehaviorTreeNodeAddAndFreeze extendedTypeAddition;

   public BehaviorTreeNodeExtensionAddAndFreeze(T nodeToAdd, T parent)
   {
      super(nodeToAdd, parent);

      if (nodeToAdd.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToAdd
       && parent.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedParent)
      {
         // This will result in recuresively performing the modification on all extended types
         extendedTypeAddition = new BehaviorTreeNodeExtensionAddAndFreeze(extendedNodeToAdd, extendedParent);
      }
      else
      {
         extendedTypeAddition = new BehaviorTreeNodeAddAndFreeze(nodeToAdd.getExtendedNode(), parent.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      extendedTypeAddition.performOperation();

      super.performOperation();
   }
}
