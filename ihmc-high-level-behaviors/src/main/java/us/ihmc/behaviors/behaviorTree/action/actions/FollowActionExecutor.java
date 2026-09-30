package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;

/**
 * Follows a person through the same controller speech uses. The leaf does not wait on this thread:
 * {@link #triggerExecution()} starts follow and later ticks read the result.
 * Reaching the standoff keeps the leaf running. Losing the person fails it.
 */
public class FollowActionExecutor extends ActionNodeExecutor<FollowActionState, FollowActionDefinition>
{
   private long followRequestId;

   public FollowActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new FollowActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      BehaviorTreeExecutor tree = rootNode.getTree();
      followRequestId = tree.startNestedFollow(definition.getTargetLabel(), definition.getClosestPerson());
      finishIfTerminal(state, followRequestId, tree.nestedFollowPhase(followRequestId));
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      finishIfTerminal(state, followRequestId, rootNode.getTree().nestedFollowPhase(followRequestId));
   }
}
