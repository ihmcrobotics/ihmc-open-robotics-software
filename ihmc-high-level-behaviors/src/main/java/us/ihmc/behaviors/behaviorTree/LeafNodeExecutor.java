package us.ihmc.behaviors.behaviorTree;

public class LeafNodeExecutor<S extends LeafNodeState<D>,
                              D extends LeafNodeDefinition>
      extends BehaviorTreeNodeExecutor<S, D>
{
   protected String cantExecuteMessage = "Not yet evaluated.";

   public LeafNodeExecutor(S state, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(state, rootNode);
   }

   /** Message to print when {@link LeafNodeState#getCanExecute()} is false, to communicate the problem to the operator. */
   public String getCantExecuteMessage()
   {
      if (state.getExecuteAfterNodeIsMissing())
         return "Can't execute: Execute after node is missing: %s".formatted(definition.getExecuteAfterLeafName());

      return cantExecuteMessage;
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getExecuteAfterNodeIsMissing())
         state.setCanExecute(false);
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
