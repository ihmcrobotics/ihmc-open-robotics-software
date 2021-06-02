package us.ihmc.behaviors.tools.behaviorTree;

import static us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * A fallback node proceeds through children left to right until they return SUCCESS.
 */
public interface FallbackNodeBasics extends BehaviorTreeControlFlowNodeBasics
{
   @Override
   public default BehaviorTreeNodeStatus tickInternal()
   {
      for (BehaviorTreeNodeBasics child : getChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tick();

         BehaviorTreeNodeBasics.checkStatusInNotNull(childStatus);

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
