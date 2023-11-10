package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionAdd<T extends BehaviorTreeNodeExtension<T, ?, ?, ?>>
      extends BehaviorTreeNodeAdd<T>
      implements BehaviorTreeModification
{
   private final BehaviorTreeNodeAdd extendedTypeAddition;

   public BehaviorTreeNodeExtensionAdd(T nodeToAdd, T parent)
   {
      super(nodeToAdd, parent);

      if (nodeToAdd.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToAdd
       && parent.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedParent)
      {
         // This will result in recursively performing the modification on all extended types
         extendedTypeAddition = new BehaviorTreeNodeExtensionAdd(extendedNodeToAdd, extendedParent);
      }
      else
      {
         extendedTypeAddition = new BehaviorTreeNodeAdd(nodeToAdd.getExtendedNode(), parent.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      extendedTypeAddition.performOperation();

      super.performOperation();
   }
}
