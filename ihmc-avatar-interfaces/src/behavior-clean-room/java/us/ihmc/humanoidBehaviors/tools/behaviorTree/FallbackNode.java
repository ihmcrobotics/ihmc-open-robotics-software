package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

public class FallbackNode extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public BehaviorTreeNodeStatus tick()
   {
      for (BehaviorTreeNode child : getChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tick();

         if (childStatus == RUNNING)
         {
            return RUNNING;
         }
         else if (childStatus == SUCCESS)
         {
            return SUCCESS;
         }

         // FAILURE, continue
      }

      return FAILURE;
   }
}
