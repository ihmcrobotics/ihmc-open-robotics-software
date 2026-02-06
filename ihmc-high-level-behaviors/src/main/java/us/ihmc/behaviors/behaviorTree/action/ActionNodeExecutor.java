package us.ihmc.behaviors.behaviorTree.action;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;

/**
 * Base template for a robot action, like a hand pose or a walk goal.
 */
public abstract class ActionNodeExecutor<S extends ActionNodeState<D>,
                                         D extends ActionNodeDefinition>
      extends LeafNodeExecutor<S, D>
{
   public ActionNodeExecutor(S state, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(state, rootNode);
   }

   /** Trigger the action to begin executing. Called once per execution. */
   public void triggerExecution()
   {
      super.triggerExecution();

      state.setNominalExecutionDuration(0.0);
      state.setElapsedExecutionTime(0.0);
      state.getCommandedTrajectory().accessValue().clear();
      state.getCommandedJointTrajectories().clear(0);
   }
}
