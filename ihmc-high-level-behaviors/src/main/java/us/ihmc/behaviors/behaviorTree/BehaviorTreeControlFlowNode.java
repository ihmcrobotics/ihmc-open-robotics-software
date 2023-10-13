package us.ihmc.behaviors.behaviorTree;

import java.util.ArrayList;

/**
 * Add default ArrayList storage of children nodes for a control flow node.
 */
public abstract class BehaviorTreeControlFlowNode extends BehaviorTreeNode implements BehaviorTreeControlFlowNodeBasics
{
   private final ArrayList<BehaviorTreeNodeBasics> children = new ArrayList<>();
   private boolean hasBeenClocked = false;

   @Override
   public ArrayList<BehaviorTreeNodeBasics> getChildren()
   {
      return children;
   }

   @Override
   public <T extends BehaviorTreeNodeBasics> T addChild(T child)
   {
      children.add(child);
      return child;
   }

   @Override
   public boolean getHasBeenClocked()
   {
      return hasBeenClocked;
   }

   @Override
   public void setHasBeenClocked(boolean hasBeenClocked)
   {
      this.hasBeenClocked = hasBeenClocked;
   }
}
