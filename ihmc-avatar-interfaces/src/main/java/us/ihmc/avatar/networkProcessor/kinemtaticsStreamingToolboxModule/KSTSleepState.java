package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.robotics.stateMachine.core.State;

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
         tools.pollInputCommand();
         tools.getCommandInputManager().clearAllCommands();
         ikController.update();
      }
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return ikController.hasBeenInitialized();
   }
}
