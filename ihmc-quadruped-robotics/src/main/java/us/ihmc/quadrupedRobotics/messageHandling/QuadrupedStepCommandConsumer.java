package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.CommandConsumerWithDelayBuffers;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;

public class QuadrupedStepCommandConsumer
{
   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final CommandConsumerWithDelayBuffers commandConsumerWithDelayBuffers;

   public QuadrupedStepCommandConsumer(CommandInputManager commandInputManager, QuadrupedStepMessageHandler stepMessageHandler,
                                       QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory managerFactory)
   {
      this.stepMessageHandler = stepMessageHandler;
      this.commandConsumerWithDelayBuffers = new CommandConsumerWithDelayBuffers(commandInputManager,
                                                                                 controllerToolbox.getRuntimeEnvironment().getRobotTimestamp());
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
   }
}
