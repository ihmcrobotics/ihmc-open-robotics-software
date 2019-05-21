package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.CommandConsumerWithDelayBuffers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
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
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(PlanarRegionsListCommand.class))
      {
         balanceManager.handlePlanarRegionsListCommand(commandConsumerWithDelayBuffers.pollNewestCommand(PlanarRegionsListCommand.class));
         commandConsumerWithDelayBuffers.clearCommands(PlanarRegionsListCommand.class);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(PauseWalkingCommand.class))
      {
         stepMessageHandler.handlePauseWalkingCommand(commandConsumerWithDelayBuffers.pollNewestCommand(PauseWalkingCommand.class));
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(AbortWalkingCommand.class))
      {
         commandConsumerWithDelayBuffers.pollNewestCommand(AbortWalkingCommand.class);
         stepMessageHandler.clearUpcomingSteps();
      }
   }

   public void consumeBodyCommands()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(QuadrupedBodyOrientationCommand.class))
      {
         bodyOrientationManager.handleBodyOrientationCommand(commandConsumerWithDelayBuffers.pollNewestCommand(QuadrupedBodyOrientationCommand.class));
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(QuadrupedBodyTrajectoryCommand.class))
      {
         QuadrupedBodyTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(QuadrupedBodyTrajectoryCommand.class);
         balanceManager.handleBodyTrajectoryCommand(command);
         bodyOrientationManager.handleBodyTrajectoryCommand(command);
      }
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(QuadrupedBodyHeightCommand.class))
      {
         balanceManager.handleBodyHeightCommand(commandConsumerWithDelayBuffers.pollNewestCommand(QuadrupedBodyHeightCommand.class));
      }
   }
}
