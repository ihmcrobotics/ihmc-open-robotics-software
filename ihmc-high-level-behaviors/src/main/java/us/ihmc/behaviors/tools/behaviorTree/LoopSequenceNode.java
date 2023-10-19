package us.ihmc.behaviors.tools.behaviorTree;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * Behavior tree sequence control flow node that loops around from rightmost to leftmost child.
 */
public class LoopSequenceNode extends SequenceNode
{
   private int currentChild = 0;

   public LoopSequenceNode()
   {
      setType(LoopSequenceNode.class);
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      super.tickInternal();

      if (currentChild >= getChildren().size())
      {
         currentChild = 0; // this loops back to first child
      }

      BehaviorTreeNodeBasics.checkStatusInNotNull(getChildren().get(currentChild++).tick());

      return RUNNING;
   }
}
