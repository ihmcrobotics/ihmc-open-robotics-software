package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.Timer;

public class WaitDurationActionExecutor extends ActionNodeExecutor<WaitDurationActionState, WaitDurationActionDefinition>
{
   private final WaitDurationActionState state;
   private final Timer executionTimer = new Timer();

   public WaitDurationActionExecutor(long id)
   {
      state = new WaitDurationActionState(id, ROS2ActorDesignation.ROBOT);
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
      state.setIsExecuting(executionTimer.isRunning(getDefinition().getWaitDuration()));

      state.setActionIndex(state.getActionIndex());
      state.setNominalExecutionDuration(getDefinition().getWaitDuration());
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }

   @Override
   public WaitDurationActionState getState()
   {
      return state;
   }
}
