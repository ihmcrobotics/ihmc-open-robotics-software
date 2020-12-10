package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import java.util.ArrayList;

/**
 * Add default ArrayList storage of children nodes for a control flow node.
 */
public abstract class BehaviorTreeControlFlowNodeBasics implements BehaviorTreeControlFlowNode
{
   private final ArrayList<BehaviorTreeNode> children = new ArrayList<>();
   private boolean hasBeenClocked = false;

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

   @Override
   public void clock()
   {
      hasBeenClocked = true;
      for (BehaviorTreeNode child : getChildren())
      {
         child.clock();
      }
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (!hasBeenClocked)
      {
         clock();
      }
      hasBeenClocked = false;

      return null;
   }
}
