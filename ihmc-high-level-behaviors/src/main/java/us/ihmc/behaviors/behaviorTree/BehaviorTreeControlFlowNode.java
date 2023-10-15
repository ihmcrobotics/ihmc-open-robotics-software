package us.ihmc.behaviors.behaviorTree;

import java.util.ArrayList;

/**
 * A behavior tree control flow node can have children.
 * Add default ArrayList storage of children nodes for a control flow node.
 */
public abstract class BehaviorTreeControlFlowNode extends BehaviorTreeNode
{
   private final ArrayList<BehaviorTreeNode> children = new ArrayList<>();
   private boolean hasBeenClocked = false;

   public BehaviorTreeNodeStatus tick()
   {
      if (!getHasBeenClocked())
      {
         clock();
      }
      setHasBeenClocked(false);

      return super.tick();
   }


   public void clock()
   {
      setHasBeenClocked(true);
      for (BehaviorTreeNode child : getChildren())
      {
         child.clock();
      }
      super.clock();
   }

   public ArrayList<BehaviorTreeNode> getChildren()
   {
      return children;
   }

   public <T extends BehaviorTreeNode> T addChild(T child)
   {
      children.add(child);
      return child;
   }

   public boolean getHasBeenClocked()
   {
      return hasBeenClocked;
   }

   public void setHasBeenClocked(boolean hasBeenClocked)
   {
      this.hasBeenClocked = hasBeenClocked;
   }
}
