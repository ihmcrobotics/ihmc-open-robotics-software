package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import java.util.ArrayList;

public abstract class BehaviorTreeControlFlowNodeBasics implements BehaviorTreeControlFlowNode
{
   private final ArrayList<BehaviorTreeNode> children = new ArrayList<>();

   protected ArrayList<BehaviorTreeNode> getChildren()
   {
      return children;
   }

   @Override
   public <T extends BehaviorTreeNode> T addChild(T child)
   {
      children.add(child);
      return child;
   }
}
