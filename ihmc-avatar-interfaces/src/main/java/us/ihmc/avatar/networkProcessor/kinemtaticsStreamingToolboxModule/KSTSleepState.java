package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.robotics.stateMachine.core.State;

public class KSTSleepState implements State
{
   private final HumanoidKinematicsToolboxController ikController;

   public KSTSleepState(KSTTools tools)
   {
      ikController = tools.getIKController();
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      if (!ikController.hasBeenInitialized())
         ikController.update();
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
