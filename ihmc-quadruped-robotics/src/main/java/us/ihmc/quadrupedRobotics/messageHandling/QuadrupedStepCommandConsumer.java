package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.CommandConsumerWithDelayBuffers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyHeightCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyOrientationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;

public class QuadrupedStepCommandConsumer
{
   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final CommandConsumerWithDelayBuffers commandConsumerWithDelayBuffers;

   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;

   public QuadrupedStepCommandConsumer(CommandInputManager commandInputManager, QuadrupedStepMessageHandler stepMessageHandler,
                                       QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory managerFactory)
   {
      this.stepMessageHandler = stepMessageHandler;
      this.commandConsumerWithDelayBuffers = new CommandConsumerWithDelayBuffers(commandInputManager,
                                                                                 controllerToolbox.getRuntimeEnvironment().getRobotTimestamp());

      balanceManager = managerFactory.getOrCreateBalanceManager();
      bodyOrientationManager = managerFactory.getOrCreateBodyOrientationManager();
   }

   public void update()
   {
      commandConsumerWithDelayBuffers.update();
   }

   public void consumeFootCommands()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(QuadrupedTimedStepListCommand.class))
      {
         stepMessageHandler.handleQuadrupedTimedStepListCommand(commandConsumerWithDelayBuffers.pollNewestCommand(QuadrupedTimedStepListCommand.class));
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(SoleTrajectoryCommand.class))
      {
         stepMessageHandler.handleSoleTrajectoryCommand(commandConsumerWithDelayBuffers.pollNewCommands(SoleTrajectoryCommand.class));
      }
   }

   public void consumeBodyCommands()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(QuadrupedBodyOrientationCommand.class))
      {
         bodyOrientationManager.handleBodyOrientationCommand(commandConsumerWithDelayBuffers.pollNewestCommand(QuadrupedBodyOrientationCommand.class));
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(QuadrupedBodyHeightCommand.class))
      {
         balanceManager.handleBodyHeightCommand(commandConsumerWithDelayBuffers.pollNewestCommand(QuadrupedBodyHeightCommand.class));
      }
   }
}
