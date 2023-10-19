package us.ihmc.behaviors.tools.behaviorTree;

import java.util.ArrayList;

/**
 * A behavior tree control flow node can have children, so provide the ability to add them.
 */
public interface BehaviorTreeControlFlowNodeBasics extends BehaviorTreeNodeBasics
{
   default BehaviorTreeNodeStatus tick()
   {
      if (!getHasBeenClocked())
      {
         clock();
      }
      setHasBeenClocked(false);

      return BehaviorTreeNodeBasics.super.tick();
   }

   default void clock()
   {
      setHasBeenClocked(true);
      for (BehaviorTreeNodeBasics child : getChildren())
      {
         child.clock();
      }
      BehaviorTreeNodeBasics.super.clock();
   }

   <T extends BehaviorTreeNodeBasics> T addChild(T child);

   ArrayList<BehaviorTreeNodeBasics> getChildren();

   void setHasBeenClocked(boolean hasBeenClocked);

   boolean getHasBeenClocked();
}
