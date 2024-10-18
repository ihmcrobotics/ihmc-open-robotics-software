package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.ObjectCarryMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import toolbox_msgs.msg.dds.KSTLoggingMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionDebug;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.IKRobotStateUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters.ClockType;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.Map;

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.SLEEP;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.STREAMING;

/**
 * The main class for setting up the IK streaming controller.
 */
public class KinematicsStreamingToolboxController extends ToolboxController
{
   public enum KSTState
   {
      SLEEP, STREAMING;

      private static final KSTState[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static KSTState fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }

   private final KSTTools tools;

   private final ExecutionTimer executionTimer = new ExecutionTimer("IKStreamingTimer", registry);
   private KSTTimeProvider timeProvider;
   private final YoDouble time = new YoDouble("time", registry);
   private final StateMachine<KSTState, State> stateMachine;

   private final KSTSleepState sleepState;
   private final KSTStreamingState streamingState;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final Class[] commandTypesToForward;

   public KinematicsStreamingToolboxController(CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager,
                                               KinematicsStreamingToolboxParameters parameters,
                                               FullHumanoidRobotModel desiredFullRobotModel,
                                               FullHumanoidRobotModelFactory fullRobotModelFactory,
                                               YoGraphicsListRegistry yoGraphicsListRegistry,
                                               YoRegistry parentRegistry)
   {
      this(commandInputManager, statusOutputManager, parameters, desiredFullRobotModel, fullRobotModelFactory, true, yoGraphicsListRegistry, parentRegistry);
   }

   public KinematicsStreamingToolboxController(CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager,
                                               KinematicsStreamingToolboxParameters parameters,
                                               FullHumanoidRobotModel desiredFullRobotModel,
                                               FullHumanoidRobotModelFactory fullRobotModelFactory,
                                               boolean runPostureOptimizer,
                                               YoGraphicsListRegistry yoGraphicsListRegistry,
                                               YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      if (parameters.getClockType() == ClockType.CPU_CLOCK)
         timeProvider = KSTTimeProvider.createCPUClockBased();
      else if (parameters.getClockType() == ClockType.FIXED_DT)
         timeProvider = KSTTimeProvider.createFixedDT(parameters.getToolboxUpdatePeriod());
      else
         throw new RuntimeException("Unknown clock type: " + parameters.getClockType());

      tools = new KSTTools(commandInputManager,
                           statusOutputManager,
                           parameters,
                           desiredFullRobotModel,
                           fullRobotModelFactory,
                           time,
                           runPostureOptimizer,
                           yoGraphicsListRegistry,
                           registry);

      // Sleep state does pretty much nothing.
      sleepState = new KSTSleepState(tools);
      // Streaming state is where the magic happens.
      streamingState = new KSTStreamingState(tools);

      stateMachine = createStateMachine(time);
      isDone.set(false);

      if (parameters.getInitialConfigurationMap() != null)
         setInitialRobotConfigurationNamedMap(parameters.getInitialConfigurationMap());

      // All the commands that are supported by the IK solver will automatically be forwarded to the IK solver.
      List<Class<? extends Command<?, ?>>> ikSolverCommands = KinematicsToolboxModule.supportedCommands();
      commandTypesToForward = commandInputManager.getListOfSupportedCommands().stream().filter(ikSolverCommands::contains).toArray(Class[]::new);
   }

   /**
    * Set the time provider for the controller. The time provider is used to get the current time for the controller.
    * <p>
    * By default, the time provider is set to a CPU clock based time provider, which is useful for real-time applications. However, for testing purposes, a
    * fixed time step time provider can be used.
    * </p>
    *
    * @param timeProvider the time provider to use for the controller.
    */
   public void setTimeProvider(KSTTimeProvider timeProvider)
   {
      this.timeProvider = timeProvider;
   }

   public void setInitialRobotConfiguration(DRCRobotModel robotModel)
   {
      tools.getIKController().setInitialRobotConfiguration(robotModel);
   }

   public void setInitialRobotConfigurationNamedMap(Map<String, Double> initialConfiguration)
   {
      tools.getIKController().setInitialRobotConfigurationNamedMap(initialConfiguration);
   }

   public void setCollisionModel(RobotCollisionModel collisionModel)
   {
      tools.getIKController().setCollisionModel(collisionModel);
   }

   public void updateKinematicsSupportRegionDebug(KinematicsToolboxSupportRegionDebug supportRegionDebug)
   {
      tools.getIKController().updateUnoptimizedStablityData(supportRegionDebug);
   }

   private StateMachine<KSTState, State> createStateMachine(DoubleProvider timeProvider)
   {
      StateMachineFactory<KSTState, State> factory = new StateMachineFactory<>(KSTState.class);
      factory.setNamePrefix("mainStateMachine").setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(SLEEP, sleepState);
      factory.addState(STREAMING, streamingState);

      factory.addDoneTransition(SLEEP, STREAMING);

      return factory.build(KSTState.SLEEP);
   }

   public void setTrajectoryMessagePublisher(WholeBodyTrajectoryMessagePublisher outputPublisher)
   {
      tools.setTrajectoryMessagerPublisher(outputPublisher);
   }

   public void setStreamingMessagePublisher(WholeBodyStreamingMessagePublisher outputPublisher)
   {
      tools.setStreamingMessagePublisher(outputPublisher);
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      timeProvider.initialize();
      time.set(timeProvider.getTime());
      return true;
   }

   @Override
   public void updateInternal()
   {
      try
      {
         executionTimer.startMeasurement();
         timeProvider.update();
         time.set(timeProvider.getTime());

         forwardCommandsToIKSolver();
         tools.update();
         stateMachine.doActionAndTransition();
      }
      catch (Throwable e)
      {
         e.printStackTrace();

         try
         {
            reportMessage(MessageTools.createControllerCrashNotificationPacket(null, e));
         }
         catch (Exception e1)
         {
            e1.printStackTrace();
         }

         isDone.set(true);
      }
      finally
      {
         executionTimer.stopMeasurement();
      }
   }

   @SuppressWarnings("unchecked")
   private void forwardCommandsToIKSolver()
   {
      for (Class commandType : commandTypesToForward)
      {
         if (tools.getCommandInputManager().isNewCommandAvailable(commandType))
         { // Forwarding commands for the IK to the IK.
            tools.getIKCommandInputManager().submitCommands(tools.getCommandInputManager().pollNewCommands(commandType));
         }
      }
   }

   public void onObjectCarryMessageReceived(ObjectCarryMessage objectCarryMessage)
   {
      tools.onObjectCarryMessageReceived(objectCarryMessage);
   }

   public void onLoggingRequestReceived(KSTLoggingMessage loggingMessage)
   {
      if (loggingMessage.getStartLogging())
         tools.getLogger().onLogRequestStart();
      else
         tools.getLogger().onLogRequestFinish();
   }

   @Override
   public void notifyToolboxStateChange(ToolboxState newState)
   {
      if (newState == ToolboxState.SLEEP)
         stateMachine.performTransition(SLEEP);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   public interface WholeBodyTrajectoryMessagePublisher
   {
      void publish(WholeBodyTrajectoryMessage messageToPublish);
   }

   public interface WholeBodyStreamingMessagePublisher
   {
      void publish(WholeBodyStreamingMessage messageToPublish);
   }

   KSTState getCurrentStateKey()
   {
      return stateMachine.getCurrentStateKey();
   }

   public void setRobotStateUpdater(IKRobotStateUpdater robotStateUpdater)
   {
      tools.setRobotStateUpdater(robotStateUpdater);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      tools.updateCapturabilityBasedStatus(newStatus);
   }

   public double getTime()
   {
      return time.getDoubleValue();
   }

   public KSTTools getTools()
   {
      return tools;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return tools.getDesiredFullRobotModel();
   }

   public interface KSTTimeProvider
   {
      void initialize();

      void update();

      double getTime();

      static KSTTimeProvider createCPUClockBased()
      {
         return new KSTTimeProvider()
         {
            private long initialTimestamp = -1L;
            private double time = 0.0;

            @Override
            public void initialize()
            {
               initialTimestamp = System.nanoTime();
               time = 0.0;
            }

            @Override
            public void update()
            {
               time = Conversions.nanosecondsToSeconds(System.nanoTime() - initialTimestamp);
            }

            @Override
            public double getTime()
            {
               return time;
            }
         };
      }

      static KSTTimeProvider createFixedDT(double dt)
      {
         return new KSTTimeProvider()
         {
            private double time = 0.0;

            @Override
            public void initialize()
            {
               time = 0.0;
            }

            @Override
            public void update()
            {
               time += dt;
            }

            @Override
            public double getTime()
            {
               return time;
            }
         };
      }
   }
}
