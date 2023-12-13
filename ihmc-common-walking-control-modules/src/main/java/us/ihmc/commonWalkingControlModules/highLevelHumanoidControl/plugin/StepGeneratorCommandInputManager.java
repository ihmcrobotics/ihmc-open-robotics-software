package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class StepGeneratorCommandInputManager implements Updatable
{
   private final CommandInputManager commandInputManager = new CommandInputManager(supportedCommands());

   private boolean isOpen = false;
   private boolean walk = false;
   private boolean isUnitVelocities = false;
   private double swingHeight = 0.1;
   private final Vector2D desiredVelocity = new Vector2D();
   private double turningVelocity = 0.0;
   private int ticksToUpdateTheEnvironment = Integer.MAX_VALUE;
   private HighLevelControllerName currentController;
   private ContinuousStepGenerator continuousStepGenerator;

   private final List<Consumer<PlanarRegionsListCommand>> planarRegionsListCommandConsumers = new ArrayList<>();
   private final AtomicReference<FootstepStatus> latestFootstepStatusReceived = new AtomicReference<>(null);
   private final AtomicReference<FootstepStatus> previousFootstepStatusReceived = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegionsListCommand> latestPlanarRegions = new AtomicReference<>(null);
   private final AtomicReference<WalkingStatus> latestWalkingStatus = new AtomicReference<>(null);
   private final AtomicReference<WalkingStatus> previousWalkingStatus = new AtomicReference<>(null);
   private final AtomicBoolean shouldSubmitNewRegions = new AtomicBoolean(true);
   private final AtomicInteger ticksSinceUpdatingTheEnvironment = new AtomicInteger(0);

   public StepGeneratorCommandInputManager()
   {
   }

   public void setCSG(ContinuousStepGenerator continuousStepGenerator)
   {
      this.continuousStepGenerator = continuousStepGenerator;
   }

   public void addPlanarRegionsListCommandConsumer(Consumer<PlanarRegionsListCommand> planarRegionsListCommandConsumer)
   {
      planarRegionsListCommandConsumers.add(planarRegionsListCommandConsumer);
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      return StepGeneratorAPIDefinition.getStepGeneratorSupportedCommands();
   }

   public void setHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
   {
      currentController = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
   }

   public void setWalkingStatus(WalkingStatusMessage message)
   {
      latestWalkingStatus.set(WalkingStatus.fromByte(message.getWalkingStatus()));
   }

   public void setFootstepStatusListener(StatusMessageOutputManager statusMessageOutputManager)
   {
      statusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, this::consumeFootstepStatus);
   }

   public void consumeFootstepStatus(FootstepStatusMessage statusMessage)
   {
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      if (status == FootstepStatus.COMPLETED)
         notifyFootstepCompleted(RobotSide.fromByte(statusMessage.getRobotSide()));
      else if (status == FootstepStatus.STARTED)
         notifyFootstepStarted();
   }

   public void notifyFootstepCompleted(RobotSide robotSide)
   {
      latestFootstepStatusReceived.set(FootstepStatus.COMPLETED);
   }

   /**
    * Notifies this generator that a footstep has been started, i.e. the foot started swinging.
    * <p>
    * It is used internally to mark the first generated footstep as unmodifiable so it does not change
    * while the swing foot is targeting it.
    * </p>
    */
   public void notifyFootstepStarted()
   {
      latestFootstepStatusReceived.set(FootstepStatus.STARTED);
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

         ticksToUpdateTheEnvironment = parameters.getTicksToUpdateTheEnvironment();
         swingHeight = parameters.getSwingHeight();

         if (continuousStepGenerator != null)
         {
            continuousStepGenerator.setFootstepTiming(parameters.getSwingDuration(), parameters.getTransferDuration());
            continuousStepGenerator.setSwingHeight(swingHeight);
            continuousStepGenerator.setFootstepsAreAdjustable(parameters.getStepsAreAdjustable());
            continuousStepGenerator.setStepWidths(parameters.getDefaultStepWidth(), parameters.getMinStepWidth(), parameters.getMaxStepWidth());
         }
      }
      commandInputManager.clearCommands(ContinuousStepGeneratorParametersCommand.class);

      // update the local planar regions
      if (commandInputManager.isNewCommandAvailable(PlanarRegionsListCommand.class))
      {
         latestPlanarRegions.set(commandInputManager.pollNewestCommand(PlanarRegionsListCommand.class));
      }
      commandInputManager.clearCommands(PlanarRegionsListCommand.class);

      // if the robot is standing, or we just finished a step, we should submit the newest regions
      if (latestWalkingStatus.get() == WalkingStatus.COMPLETED || latestFootstepStatusReceived.get() == FootstepStatus.COMPLETED)
         shouldSubmitNewRegions.set(true);

      // if the contact state just changed, we should submit the newest regions
      if (latestFootstepStatusReceived.get() != previousFootstepStatusReceived.get())
         shouldSubmitNewRegions.set(true);

      // If the regions are old, update them
      if (ticksSinceUpdatingTheEnvironment.get() > ticksToUpdateTheEnvironment)
         shouldSubmitNewRegions.set(true);

      // submit the new planar regions
      if (isOpen && shouldSubmitNewRegions.getAndSet(false) && latestPlanarRegions.get() != null)
      {
         PlanarRegionsListCommand command = latestPlanarRegions.getAndSet(null);

         if (command != null)
         {
            for (int i = 0; i < planarRegionsListCommandConsumers.size(); i++)
               planarRegionsListCommandConsumers.get(i).accept(command);

            ticksSinceUpdatingTheEnvironment.set(0);
         }

         ticksSinceUpdatingTheEnvironment.incrementAndGet();
      }

      previousWalkingStatus.set(latestWalkingStatus.get());
      previousFootstepStatusReceived.set(latestFootstepStatusReceived.get());

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

   public DoubleProvider createSwingHeightProvider()
   {
      return () -> swingHeight;
   }
}
