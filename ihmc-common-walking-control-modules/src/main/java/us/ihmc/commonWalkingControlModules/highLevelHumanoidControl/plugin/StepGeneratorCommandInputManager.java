package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredTurningVelocityProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.DesiredVelocityProvider;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.yoVariables.providers.BooleanProvider;

import java.util.ArrayList;
import java.util.List;

public class StepGeneratorCommandInputManager implements Updatable
{
   private final CommandInputManager commandInputManager = new CommandInputManager(supportedCommands());

   private boolean isOpen = false;
   private boolean walk = false;
   private boolean isUnitVelocities = false;
   private final Vector2D desiredVelocity = new Vector2D();
   private double turningVelocity = 0.0;
   private HighLevelControllerName currentController;
   private ContinuousStepGenerator continuousStepGenerator;

   public StepGeneratorCommandInputManager()
   {
   }

   public void setCSG(ContinuousStepGenerator continuousStepGenerator)
   {
      this.continuousStepGenerator = continuousStepGenerator;
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(ContinuousStepGeneratorParametersCommand.class);
      commands.add(ContinuousStepGeneratorInputCommand.class);
      commands.add(PlanarRegionsListCommand.class);
      return commands;
   }

   public void setHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
   {
      currentController = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
   }

   @Override
   public void update(double time)
   {
      isOpen = currentController == HighLevelControllerName.WALKING || currentController == HighLevelControllerName.CUSTOM1;

      if (commandInputManager.isNewCommandAvailable(ContinuousStepGeneratorInputCommand.class))
      {
         ContinuousStepGeneratorInputCommand command = commandInputManager.pollNewestCommand(ContinuousStepGeneratorInputCommand.class);
         desiredVelocity.setX(command.getForwardVelocity());
         desiredVelocity.setY(command.getLateralVelocity());
         turningVelocity = command.getTurnVelocity();
         isUnitVelocities = command.isUnitVelocities();
         walk = command.isWalk();
      }
      commandInputManager.clearCommands(ContinuousStepGeneratorInputCommand.class);

      if (commandInputManager.isNewCommandAvailable(ContinuousStepGeneratorParametersCommand.class))
      {
         ContinuousStepGeneratorParametersCommand command = commandInputManager.pollNewestCommand(ContinuousStepGeneratorParametersCommand.class);
         ContinuousStepGeneratorParameters parameters = command.getParameters();

         if (continuousStepGenerator != null)
         {
            continuousStepGenerator.setFootstepTiming(parameters.getSwingDuration(), parameters.getTransferDuration());
            continuousStepGenerator.setSwingHeight(parameters.getSwingHeight());
            continuousStepGenerator.setFootstepsAreAdjustable(parameters.getStepsAreAdjustable());
            continuousStepGenerator.setStepWidths(parameters.getDefaultStepWidth(), parameters.getMinStepWidth(), parameters.getMaxStepWidth());
         }
      }
      commandInputManager.clearCommands(ContinuousStepGeneratorParametersCommand.class);

      if (!isOpen)
         walk = false;
   }

   public boolean isOpen()
   {
      return isOpen;
   }

   public DesiredVelocityProvider createDesiredVelocityProvider()
   {
      return new DesiredVelocityProvider()
      {
         @Override
         public Vector2DReadOnly getDesiredVelocity()
         {
            return desiredVelocity;
         }

         @Override
         public boolean isUnitVelocity()
         {
            return isUnitVelocities;
         }
      };
   }

   public DesiredTurningVelocityProvider createDesiredTurningVelocityProvider()
   {
      return new DesiredTurningVelocityProvider()
      {
         @Override
         public double getTurningVelocity()
         {
            return turningVelocity;
         }

         @Override
         public boolean isUnitVelocity()
         {
            return isUnitVelocities;
         }
      };
   }

   public BooleanProvider createWalkInputProvider()
   {
      return () -> walk;
   }
}
