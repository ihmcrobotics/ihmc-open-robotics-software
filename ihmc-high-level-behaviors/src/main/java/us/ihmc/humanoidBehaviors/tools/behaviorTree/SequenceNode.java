package us.ihmc.humanoidBehaviors.tools.behaviorTree;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * A sequence node proceeds through children left to right while they are SUCCESSful.
 */
public class SequenceNode extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public BehaviorTreeNodeStatus tick()
   {
      super.tick();

      for (BehaviorTreeNode child : getChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tick();

         BehaviorTreeNode.checkStatusInNotNull(childStatus);

         if (childStatus == RUNNING)
         {
            return RUNNING;
         }
         else if (childStatus == FAILURE)
         {
            return FAILURE;
         }

         // SUCCESS, continue
      }

      return SUCCESS;
   }
}
