package us.ihmc.behaviors.behaviorTree;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * A sequence node proceeds through children left to right while they are SUCCESSful.
 */
public interface SequenceNodeBasics extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public default BehaviorTreeNodeStatus tickInternal()
   {
      for (BehaviorTreeNodeBasics child : getChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tick();

         BehaviorTreeNodeBasics.checkStatusIsNotNull(childStatus);

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
