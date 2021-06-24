package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.CommandConsumerWithDelayBuffers;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PushRecoveryResultCommand;
import us.ihmc.yoVariables.variable.YoDouble;

public class PushRecoveryCommandConsumer
{
   private final CommandConsumerWithDelayBuffers commandConsumerWithDelayBuffers;

   public PushRecoveryCommandConsumer(CommandInputManager commandInputManager,  YoDouble yoTime)
   {
      this.commandConsumerWithDelayBuffers = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
   }

   public void update()
   {
      commandConsumerWithDelayBuffers.update();
   }

   public void consumePushRecoveryResultCommand(MultiStepPushRecoveryController pushRecoveryModule)
   {
      if (!commandConsumerWithDelayBuffers.isNewCommandAvailable(PushRecoveryResultCommand.class))
         return;
      pushRecoveryModule.consumePushRecoveryResultCommand(commandConsumerWithDelayBuffers.pollNewestCommand(PushRecoveryResultCommand.class));
   }
}
