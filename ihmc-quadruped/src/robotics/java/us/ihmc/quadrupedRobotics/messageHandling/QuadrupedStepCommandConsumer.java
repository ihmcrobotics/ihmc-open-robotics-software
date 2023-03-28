package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.CommandConsumerWithDelayBuffers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFootStates;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedStepCommandConsumer
{
   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final CommandConsumerWithDelayBuffers commandConsumerWithDelayBuffers;

   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedFeetManager feetManager;

   public QuadrupedStepCommandConsumer(CommandInputManager commandInputManager, QuadrupedStepMessageHandler stepMessageHandler,
                                       QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory managerFactory)
   {
      this.stepMessageHandler = stepMessageHandler;

      List<Class<? extends Command<?, ?>>> commandsToRegister = new ArrayList<>();
      commandsToRegister.add(QuadrupedTimedStepListCommand.class);
      commandsToRegister.add(SoleTrajectoryCommand.class);
      commandsToRegister.add(PlanarRegionsListCommand.class);
      commandsToRegister.add(PauseWalkingCommand.class);
      commandsToRegister.add(AbortWalkingCommand.class);
      commandsToRegister.add(QuadrupedFootLoadBearingCommand.class);
      commandsToRegister.add(QuadrupedBodyOrientationCommand.class);
      commandsToRegister.add(QuadrupedBodyTrajectoryCommand.class);
      commandsToRegister.add(QuadrupedBodyHeightCommand.class);

      commandConsumerWithDelayBuffers = new CommandConsumerWithDelayBuffers(commandInputManager, commandsToRegister, controllerToolbox.getRuntimeEnvironment().getRobotTimestamp());

      balanceManager = managerFactory.getOrCreateBalanceManager();
      bodyOrientationManager = managerFactory.getOrCreateBodyOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
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
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(QuadrupedFootLoadBearingCommand.class))
      {
         List<QuadrupedFootLoadBearingCommand> commands = commandConsumerWithDelayBuffers.pollNewCommands(QuadrupedFootLoadBearingCommand.class);
         for (int i = 0; i < commands.size(); i++)
         {
            RobotQuadrant robotQuadrant = commands.get(i).getRobotQuadrant();
            if(feetManager.getCurrentState(robotQuadrant) == QuadrupedFootStates.MOVE_VIA_WAYPOINTS)
            {
               feetManager.requestContact(robotQuadrant);
            }
         }
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
