package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
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
      //ikController.requestInitialize();
   }

   @Override
   public void doAction(double timeInState)
   {
      tools.getCommandInputManager().clearCommands(KinematicsStreamingToolboxInputCommand.class);
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return true; //ikController.hasBeenInitialized();
   }
}
