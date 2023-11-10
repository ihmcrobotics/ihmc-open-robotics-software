package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionClear implements BehaviorTreeModification
{
   private final BehaviorTreeNodeExtension<?, ?, ?, ?> nodeToClear;

   public BehaviorTreeNodeExtensionClear(BehaviorTreeNodeExtension<?, ?, ?, ?> nodeToClear)
   {
      this.nodeToClear = nodeToClear;
   }

   @Override
   public void performOperation()
   {
      clearChildren(nodeToClear);
   }

   public static void clearChildren(BehaviorTreeNodeExtension<?, ?, ?, ?> nodeToClear)
   {
      BehaviorTreeNodeClear.clearChildren(nodeToClear);
      if (nodeToClear.getState() == nodeToClear.getExtendedNode())
         BehaviorTreeNodeClear.clearChildren(nodeToClear.getState());
      BehaviorTreeNodeClear.clearChildren(nodeToClear.getDefinition());
   }
}

