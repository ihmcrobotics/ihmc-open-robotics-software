package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.robotics.stateMachine.core.State;

public class JustWaitState implements State
{
   private final ControllerFootstepQueueMonitor controllerQueueMonitor;
   private boolean isDone;

   public JustWaitState(ControllerFootstepQueueMonitor controllerQueueMonitor)
   {
      this.controllerQueueMonitor = controllerQueueMonitor;
   }

   @Override
   public void onEntry()
   {
      isDone = false;
   }

   @Override
   public void doAction(double timeInState)
   {
      if (controllerQueueMonitor.getControllerFootstepQueue().isEmpty())
      {
         isDone = true;
      }
   }

   @Override
   public void onExit(double timeInState)
   {

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return isDone;
   }
}
