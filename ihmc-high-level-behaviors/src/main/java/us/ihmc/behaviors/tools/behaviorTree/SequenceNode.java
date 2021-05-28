package us.ihmc.behaviors.tools.behaviorTree;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * A sequence node proceeds through children left to right while they are SUCCESSful.
 */
public class SequenceNode extends BehaviorTreeControlFlowNode
{
   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      super.tickInternal();

      for (BehaviorTreeNodeBasics child : getChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tick();

         BehaviorTreeNodeBasics.checkStatusInNotNull(childStatus);

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
