package us.ihmc.behaviors.behaviorTree.control;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;

public class CheckpointNodeExecutor extends LeafNodeExecutor<CheckpointNodeState, CheckpointNodeDefinition>
{
   public CheckpointNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new CheckpointNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false); // Completes immediately
   }
}
