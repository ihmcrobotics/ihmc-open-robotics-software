package us.ihmc.behaviors.behaviorTree;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * Behavior tree sequence control flow node that loops around from rightmost to leftmost child.
 */
public class LoopSequenceNode extends SequenceNode
{
   private int currentChild = 0;

   public LoopSequenceNode()
   {

   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      if (currentChild >= getChildren().size())
      {
         currentChild = 0; // this loops back to first child
      }

      getChildren().get(currentChild++).tick();

      return RUNNING;
   }
}
