package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class LeafNodeExecutor<S extends LeafNodeState<D>,
                              D extends LeafNodeDefinition>
      extends BehaviorTreeNodeExecutor<S, D>
{
   public LeafNodeExecutor(S state)
   {
      super(state);
   }

   /** Message to print when {@link LeafNodeState#getCanExecute()} is false, to communicate the problem to the operator. */
   public String getCantExecuteMessage()
   {
      return "";
   }

   /** Trigger the action to begin executing. Called once per execution. */
   public void triggerExecution()
   {
      state.setIsExecuting(true);
      state.setFailed(false);
   }

   /** Called every tick only when this action is executing. */
   public void updateCurrentlyExecuting()
   {

   }
}
