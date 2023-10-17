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

   public BehaviorTreeNodeStatus tickInternal()
   {
      for (BehaviorTreeNodeExecutor child : getChildren())
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
