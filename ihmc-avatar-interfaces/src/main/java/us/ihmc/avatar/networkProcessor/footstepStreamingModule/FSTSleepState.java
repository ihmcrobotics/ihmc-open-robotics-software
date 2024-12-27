package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxInputCommand;
import us.ihmc.robotics.stateMachine.core.State;

public class FSTSleepState implements State
{
   private final FSTTools tools;

   public FSTSleepState(FSTTools tools)
   {
      this.tools = tools;
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      tools.getCommandInputManager().clearCommands(FootstepStreamingToolboxInputCommand.class);
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return true;
   }
}
