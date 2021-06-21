package us.ihmc.behaviors.tools.behaviorTree;

import java.util.ArrayList;

/**
 * A behavior tree control flow node can have children, so provide the ability to add them.
 */
public interface BehaviorTreeControlFlowNodeBasics extends BehaviorTreeNodeBasics
{
   public default BehaviorTreeNodeStatus tick()
   {
      if (!getHasBeenClocked())
      {
         clock();
      }
      setHasBeenClocked(false);

      return BehaviorTreeNodeBasics.super.tick();
   }

   public default void clock()
   {
      setHasBeenClocked(true);
      for (BehaviorTreeNodeBasics child : getChildren())
      {
         child.clock();
      }
      BehaviorTreeNodeBasics.super.clock();
   }

   public abstract <T extends BehaviorTreeNodeBasics> T addChild(T child);

   public abstract ArrayList<BehaviorTreeNodeBasics> getChildren();

   public abstract void setHasBeenClocked(boolean hasBeenClocked);

   public abstract boolean getHasBeenClocked();
}
