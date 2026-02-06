package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;

public class CheckPointNodeExecutor extends LeafNodeExecutor<CheckPointNodeState, CheckPointNodeDefinition>
{
   public CheckPointNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new CheckPointNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false); // Completes immediately
   }
}
