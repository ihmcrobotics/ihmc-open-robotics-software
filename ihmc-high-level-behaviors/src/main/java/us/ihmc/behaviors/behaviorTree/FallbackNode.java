package us.ihmc.behaviors.behaviorTree;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * A fallback node proceeds through children left to right until they return SUCCESS.
 */
public class FallbackNode extends LocalOnlyBehaviorTreeNodeExecutor
{
   public FallbackNode()
   {

   }

   public BehaviorTreeNodeStatus determineStatus()
   {
      for (LocalOnlyBehaviorTreeNodeExecutor child : getLocalOnlyChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tickAndGetStatus();

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
