package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.HighLevelStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.commons.Conversions;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.tools.NonWallTimer;

/**
 * Sends one request for the definition's controller state, then finishes once the controller reports
 * that state. Fails if the controller is not there within the timeout, for example because the
 * transition is not allowed from the current state.
 */
public class HighLevelStateActionExecutor extends ActionNodeExecutor<HighLevelStateActionState, HighLevelStateActionDefinition>
{
   private final NonWallTimer executionTimer = new NonWallTimer();

   public HighLevelStateActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new HighLevelStateActionState(id, rootNode.getState()), rootNode);
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

      HighLevelControllerName requested = definition.getHighLevelControllerName();
      state.getLogger().info("Requesting controller state %s".formatted(requested));

      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(requested.toByte());
      ros2ControllerHelper.publishToController(message);

      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setNominalExecutionDuration(definition.getTimeout());
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());

      HighLevelControllerName requested = definition.getHighLevelControllerName();
      HighLevelControllerName current = controllerStatusTracker.getLatestKnownState();
      if (current == requested)
      {
         state.setIsExecuting(false);
      }
      else if (!executionTimer.isRunning(definition.getTimeout()))
      {
         state.getLogger().error("Controller is in %s, not %s, after %.1f s".formatted(current, requested, definition.getTimeout()));
         state.setIsExecuting(false);
         state.setFailed(true);
      }
   }
}
