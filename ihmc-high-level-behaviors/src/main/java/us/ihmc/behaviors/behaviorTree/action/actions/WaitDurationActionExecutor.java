package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.commons.Conversions;
import us.ihmc.tools.NonWallTimer;

public class WaitDurationActionExecutor extends ActionNodeExecutor<WaitDurationActionState, WaitDurationActionDefinition>
{
   private final NonWallTimer executionTimer = new NonWallTimer();

   public WaitDurationActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new WaitDurationActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      executionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Waiting for %.2f s...".formatted(definition.getWaitDuration()));

      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(executionTimer.isRunning(definition.getWaitDuration()));

      state.setNominalExecutionDuration(definition.getWaitDuration());
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }
}
