package us.ihmc.behaviors.behaviorTree;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * A sequence node proceeds through children left to right while they are SUCCESSful.
 */
public class SequenceNode extends BehaviorTreeNodeState
{
   private final BehaviorTreeNodeDefinition definition = new BehaviorTreeNodeDefinition();

   public SequenceNode()
   {

   }

   public BehaviorTreeNodeStatus tickInternal()
   {
      for (BehaviorTreeNodeState child : getChildren())
      {
         BehaviorTreeNodeStatus childStatus = child.tick();

         BehaviorTreeNodeState.checkStatusIsNotNull(childStatus);

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

   @Override
   public BehaviorTreeNodeDefinition getDefinition()
   {
      return definition;
   }
}
