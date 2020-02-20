package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

public class NonReactiveLoopSequenceNode extends SequenceNode
{
   private int currentChild = 0;

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (currentChild >= getChildren().size())
      {
         currentChild = 0; // this loops back to first child
      }

      getChildren().get(currentChild++).tick();

      return RUNNING;
   }
}
