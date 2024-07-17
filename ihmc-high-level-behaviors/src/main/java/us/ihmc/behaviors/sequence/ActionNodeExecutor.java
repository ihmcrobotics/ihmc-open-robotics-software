package us.ihmc.behaviors.sequence;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;

/**
 * Base template for a robot action, like a hand pose or a walk goal.
 */
public abstract class ActionNodeExecutor<S extends ActionNodeState<D>,
                                         D extends ActionNodeDefinition>
      extends BehaviorTreeNodeExecutor<S, D>
{
   public ActionNodeExecutor(S state)
   {
      super(state);
   }

   /** Trigger the action to begin executing. Called once per execution. */
   public void triggerActionExecution()
   {
      getState().setIsExecuting(true);
      getState().setFailed(false);
      getState().setNominalExecutionDuration(0.0);
      getState().setElapsedExecutionTime(0.0);
      getState().getCommandedTrajectory().accessValue().clear();
      getState().getCommandedJointTrajectories().clear(0);
   }

   /** Called every tick only when this action is executing. */
   public void updateCurrentlyExecuting()
   {

   }
}
