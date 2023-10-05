package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.tools.Timer;

public class WaitDurationActionExecutor implements BehaviorActionExecutor
{
   private final WaitDurationActionState state = new WaitDurationActionState();
   private final WaitDurationActionDefinition definition = state.getDefinition();
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public WaitDurationActionExecutor()
   {

   }

   @Override
   public void update(int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {

   }

   @Override
   public void triggerActionExecution()
   {
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      isExecuting = executionTimer.isRunning(definition.getWaitDuration());

      executionStatusMessage.setActionIndex(state.getActionIndex());
      executionStatusMessage.setNominalExecutionDuration(definition.getWaitDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
   }

   @Override
   public WaitDurationActionState getState()
   {
      return state;
   }

   @Override
   public WaitDurationActionDefinition getDefinition()
   {
      return definition;
   }
}
