package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import controller_msgs.FootstepDataMessage;
import us.ihmc.avatar.networkProcessor.footstepStreamingModule.FootstepStreamingToolboxParameters.ClockType;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.avatar.networkProcessor.footstepStreamingModule.FootstepStreamingToolboxController.FSTState.SLEEP;
import static us.ihmc.avatar.networkProcessor.footstepStreamingModule.FootstepStreamingToolboxController.FSTState.STREAMING;

/**
 * The main class for setting up the footstep streaming controller.
 */
public class FootstepStreamingToolboxController extends ToolboxController
{
   public enum FSTState
   {
      SLEEP, STREAMING;

      private static final FSTState[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static FSTState fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }

   private final FSTTools tools;

   private final ExecutionTimer executionTimer = new ExecutionTimer("FSTTimer", registry);
   private FSTTimeProvider timeProvider;
   private final YoDouble time = new YoDouble("time", registry);
   private final StateMachine<FSTState, State> stateMachine;

   private final FSTSleepState sleepState;
   private final FSTStreamingState streamingState;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   public FootstepStreamingToolboxController(CommandInputManager commandInputManager,
                                             StatusMessageOutputManager statusOutputManager,
                                             FootstepStreamingToolboxParameters parameters,
                                             FullHumanoidRobotModel desiredFullRobotModel,
                                             FullHumanoidRobotModelFactory fullRobotModelFactory,
                                             YoGraphicsListRegistry yoGraphicsListRegistry,
                                             YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      if (parameters.getClockType() == ClockType.CPU_CLOCK)
         timeProvider = FSTTimeProvider.createCPUClockBased();
      else if (parameters.getClockType() == ClockType.FIXED_DT)
         timeProvider = FSTTimeProvider.createFixedDT(parameters.getToolboxUpdatePeriod());
      else
         throw new RuntimeException("Unknown clock type: " + parameters.getClockType());

      tools = new FSTTools(commandInputManager,
                           statusOutputManager,
                           parameters,
                           time,
                           yoGraphicsListRegistry,
                           registry);

      // Sleep state does pretty much nothing.
      sleepState = new FSTSleepState(tools);
      // Streaming state is where the magic happens.
      streamingState = new FSTStreamingState(tools);

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
   public void setTimeProvider(FSTTimeProvider timeProvider)
   {
      this.timeProvider = timeProvider;
   }

   private StateMachine<FSTState, State> createStateMachine(DoubleProvider timeProvider)
   {
      StateMachineFactory<FSTState, State> factory = new StateMachineFactory<>(FSTState.class);
      factory.setNamePrefix("mainStateMachine").setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(SLEEP, sleepState);
      factory.addState(STREAMING, streamingState);

      factory.addDoneTransition(SLEEP, STREAMING);

      return factory.build(SLEEP);
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

   FSTState getCurrentStateKey()
   {
      return stateMachine.getCurrentStateKey();
   }

   public double getTime()
   {
      return time.getDoubleValue();
   }

   public interface FSTTimeProvider
   {
      void initialize();

      void update();

      double getTime();

      static FSTTimeProvider createCPUClockBased()
      {
         return new FSTTimeProvider()
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

      static FSTTimeProvider createFixedDT(double dt)
      {
         return new FSTTimeProvider()
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
