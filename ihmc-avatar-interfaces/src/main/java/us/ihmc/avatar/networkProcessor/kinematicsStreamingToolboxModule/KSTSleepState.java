package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.robotics.stateMachine.core.State;

/**
 * Idle state for the streaming toolbox state machine.
 * It keeps reinitializing the IK controller until it is ready, while dropping queued streaming inputs.
 */
public class KSTSleepState implements State
{
   private final HumanoidKinematicsToolboxController ikController;
   private final KSTTools tools;

   public KSTSleepState(KSTTools tools)
   {
      this.tools = tools;
      ikController = tools.getIKController();
   }

   @Override
   public void onEntry()
   {
      ikController.requestInitialize();
   }

   @Override
   public void doAction(double timeInState)
   {
      if (!ikController.hasBeenInitialized())
      {
         tools.getCommandInputManager().clearCommands(KinematicsStreamingToolboxInputCommand.class);
         ikController.update();
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return ikController.hasBeenInitialized();
   }
}
