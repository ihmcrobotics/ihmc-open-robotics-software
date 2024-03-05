package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.SLEEP;
import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.KSTState.STREAMING;

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

   ;

   private final KSTTools tools;

   private KSTTimeProvider timeProvider = KSTTimeProvider.createCPUClockBased();
   private final YoDouble time = new YoDouble("time", registry);
   private final StateMachine<KSTState, State> stateMachine;

   private final KSTSleepState sleepState;
   private final KSTStreamingState streamingState;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   public KinematicsStreamingToolboxController(CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager,
                                               FullHumanoidRobotModel desiredFullRobotModel,
                                               FullHumanoidRobotModelFactory fullRobotModelFactory,
                                               double walkingControllerPeriod,
                                               double toolboxControllerPeriod,
                                               YoGraphicsListRegistry yoGraphicsListRegistry,
                                               YoRegistry parentRegistry)
   {
      this(commandInputManager,
           statusOutputManager,
           KinematicsStreamingToolboxParameters.defaultParameters(),
           desiredFullRobotModel,
           fullRobotModelFactory,
           walkingControllerPeriod,
           toolboxControllerPeriod,
           yoGraphicsListRegistry,
           parentRegistry);
   }

   public KinematicsStreamingToolboxController(CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager,
                                               KinematicsStreamingToolboxParameters parameters,
                                               FullHumanoidRobotModel desiredFullRobotModel,
                                               FullHumanoidRobotModelFactory fullRobotModelFactory,
                                               double walkingControllerPeriod,
                                               double toolboxControllerPeriod,
                                               YoGraphicsListRegistry yoGraphicsListRegistry,
                                               YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      tools = new KSTTools(commandInputManager,
                           statusOutputManager,
                           parameters,
                           desiredFullRobotModel,
                           fullRobotModelFactory,
                           walkingControllerPeriod,
                           toolboxControllerPeriod,
                           time,
                           yoGraphicsListRegistry,
                           registry);

      sleepState = new KSTSleepState(tools);
      streamingState = new KSTStreamingState(tools);

      stateMachine = createStateMachine(time);
      isDone.set(false);
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
         timeProvider.update();
         time.set(timeProvider.getTime());

         if (tools.getCommandInputManager().isNewCommandAvailable(KinematicsToolboxConfigurationCommand.class))
         { // Forwarding commands for the IK to the IK.
            tools.getIKCommandInputManager().submitCommands(tools.getCommandInputManager().pollNewCommands(KinematicsToolboxConfigurationCommand.class));
         }
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

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      tools.updateRobotConfigurationData(newConfigurationData);
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
