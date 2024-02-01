package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.CommandConsumerWithDelayBuffers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylSolverTrajectoryCommand;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class ExternalControlCommandConsumer
{
   private final CommandConsumerWithDelayBuffers commandConsumerWithDelayBuffers;
   private final WholeBodyConfigurationManager wholeBodyConfigurationManager;

   public ExternalControlCommandConsumer(CommandInputManager commandInputManager,
                                         WholeBodyConfigurationManager wholeBodyConfigurationManager,
                                         YoDouble yoTime)
   {
      this.wholeBodyConfigurationManager = wholeBodyConfigurationManager;

      List<Class<? extends Command<?, ?>>> commandsToRegister = new ArrayList<>();
      commandsToRegister.add(CrocoddylSolverTrajectoryCommand.class);
      commandConsumerWithDelayBuffers = new CommandConsumerWithDelayBuffers(commandInputManager, commandsToRegister, yoTime);
   }

   public void consumeExternalControlCommands()
   {
      if (commandConsumerWithDelayBuffers.isNewCommandAvailable(CrocoddylSolverTrajectoryCommand.class))
      {
         CrocoddylSolverTrajectoryCommand command = commandConsumerWithDelayBuffers.pollNewestCommand(CrocoddylSolverTrajectoryCommand.class);
         wholeBodyConfigurationManager.handleCrocoddylSolverTrajectoryCommand(command);
      }
   }
}
