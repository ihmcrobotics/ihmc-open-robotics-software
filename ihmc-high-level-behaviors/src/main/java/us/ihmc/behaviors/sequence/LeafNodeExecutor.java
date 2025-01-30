package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

public class LeafNodeExecutor<S extends LeafNodeState<D>,
                              D extends LeafNodeDefinition>
      extends BehaviorTreeNodeExecutor<S, D>
{
   private final S state;

   public LeafNodeExecutor(S state)
   {
      super(state);

      this.state = state;
   }

   public String getCantExecuteMessage()
   {
      return "";
   }

   /** Trigger the action to begin executing. Called once per execution. */
   public void triggerActionExecution()
   {
      state.setIsExecuting(true);
      state.setFailed(false);
   }

   /** Called every tick only when this action is executing. */
   public void updateCurrentlyExecuting()
   {

   }
}
