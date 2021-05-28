package us.ihmc.behaviors.tools.behaviorTree;

import java.util.ArrayList;

/**
 * Add default ArrayList storage of children nodes for a control flow node.
 */
public abstract class BehaviorTreeControlFlowNode implements BehaviorTreeControlFlowNodeBasics
{
   private final ArrayList<BehaviorTreeNodeBasics> children = new ArrayList<>();
   private boolean hasBeenClocked = false;

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
   public void clock()
   {
      hasBeenClocked = true;
      for (BehaviorTreeNodeBasics child : getChildren())
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
