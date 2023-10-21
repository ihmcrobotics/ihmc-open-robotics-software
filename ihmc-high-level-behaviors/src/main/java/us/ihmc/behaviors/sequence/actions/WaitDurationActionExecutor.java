package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.tools.Timer;

public class WaitDurationActionExecutor extends BehaviorActionExecutor
{
   private final WaitDurationActionState state = new WaitDurationActionState();
   private final WaitDurationActionDefinition definition = state.getDefinition();
   private final Timer executionTimer = new Timer();
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public WaitDurationActionExecutor(BehaviorActionSequence sequence)
   {
      super(sequence);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(executionTimer.isRunning(definition.getWaitDuration()));

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
