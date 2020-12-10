package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * Behavior tree sequence control flow node that loops around from rightmost to leftmost child.
 */
public class LoopSequenceNode extends SequenceNode
{
   private int currentChild = 0;

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      super.tick();

      if (currentChild >= getChildren().size())
      {
         currentChild = 0; // this loops back to first child
      }

      BehaviorTreeNode.checkStatusInNotNull(getChildren().get(currentChild++).tick());

      return RUNNING;
   }
}
