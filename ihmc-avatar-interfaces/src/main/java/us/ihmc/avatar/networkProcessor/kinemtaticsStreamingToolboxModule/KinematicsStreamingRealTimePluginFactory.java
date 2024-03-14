package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.AvatarControllerThreadInterface;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is meant to wrap an instance of the {@link KinematicsStreamingToolboxController} into a real-time plugin that can be used on the control computer.
 */
public class KinematicsStreamingRealTimePluginFactory
{
   private KinematicsStreamingToolboxParameters parameters;
   private IKStreamingRTControllerThread ikStreamingRTController;
   private IKStreamingRTTask ikStreamingRTTask;

   public KinematicsStreamingRealTimePluginFactory()
   {

   }

   public void setParameters(KinematicsStreamingToolboxParameters parameters)
   {
      this.parameters = parameters;
   }

   public IKStreamingRTControllerThread createRTController(String robotName,
                                                           ROS2NodeInterface ros2Node,
                                                           CommandInputManager walkingInputManager,
                                                           StatusMessageOutputManager walkingOutputManager,
                                                           FullHumanoidRobotModelFactory fullRobotModelFactory,
                                                           RobotCollisionModel collisionModel)
   {
      if (ikStreamingRTController == null)
         ikStreamingRTController = new IKStreamingRTControllerThread(robotName,
                                                                     ros2Node,
                                                                     walkingInputManager,
                                                                     walkingOutputManager,
                                                                     fullRobotModelFactory,
                                                                     collisionModel,
                                                                     parameters);
      return ikStreamingRTController;
   }

   public IKStreamingRTTask createRTTask(String prefix, double schedulerDt, FullHumanoidRobotModel masterFullRobotModel)
   {
      if (ikStreamingRTController == null)
         throw new RuntimeException("Controller has not been created yet.");
      if (ikStreamingRTTask == null)
      {
         long divisor = Math.round(parameters.getToolboxUpdatePeriod() / schedulerDt);
         if (!EuclidCoreTools.epsilonEquals(divisor * schedulerDt, parameters.getToolboxUpdatePeriod(), 1.0e-7) || divisor < 1)
         {
            throw new IllegalArgumentException("The schedulerDt (%s) does not divide the toolbox update period (%s).".formatted(schedulerDt,
                                                                                                                                parameters.getToolboxUpdatePeriod()));
         }
         ikStreamingRTTask = new IKStreamingRTTask(prefix, ikStreamingRTController, divisor, schedulerDt, masterFullRobotModel);
      }
      return ikStreamingRTTask;
   }

   public static class IKStreamingRTTask extends HumanoidRobotControlTask
   {
      private final IKStreamingRTControllerThread ikStreamingThread;

      private final long divisor;
      private final ThreadTimer timer;
      private final YoLong ticksBehindScheduled;

      private final List<Runnable> postControllerCallbacks = new ArrayList<>();
      private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();
      private long timestamp = 0;

      public IKStreamingRTTask(String prefix,
                               IKStreamingRTControllerThread ikStreamingThread,
                               long divisor,
                               double schedulerDt,
                               FullHumanoidRobotModel masterFullRobotModel)
      {
         super(divisor);
         this.divisor = divisor;
         this.ikStreamingThread = ikStreamingThread;

         //      String prefix = "Controller";
         timer = new ThreadTimer(prefix, schedulerDt * divisor, ikStreamingThread.getYoVariableRegistry());
         ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", ikStreamingThread.getYoVariableRegistry());
      }

      @Override
      protected boolean initialize()
      {
         // For when the task gets reset, so we can observe when it gets triggered.
         timer.reset();
         ticksBehindScheduled.set(0);
         return super.initialize();
      }

      private long schedulerTick = -1L;

      @Override
      protected void execute()
      {
         timer.start();
         ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
         ikStreamingThread.run();
         runAll(postControllerCallbacks);
         timer.stop();
      }

      @Override
      protected void updateMasterContext(HumanoidRobotContextData context)
      {
         runAll(schedulerThreadRunnables);
      }

      @Override
      protected void updateLocalContext(HumanoidRobotContextData context)
      {
         schedulerTick = context.getSchedulerTick();
         timestamp = context.getTimestamp();
      }

      @Override
      public void addCallbackPostTask(Runnable runnable)
      {
         postControllerCallbacks.add(runnable);
      }

      @Override
      public void addRunnableOnSchedulerThread(Runnable runnable)
      {
         schedulerThreadRunnables.add(runnable);
      }
   }

   public static class IKStreamingRTControllerThread implements AvatarControllerThreadInterface
   {
      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      private final CommandInputManager commandInputManager;
      private final StatusMessageOutputManager statusOutputManager;
      private final KinematicsStreamingToolboxController kinematicsStreamingToolboxController;

      private final AtomicBoolean receivedInput = new AtomicBoolean();
      private final YoDouble timeWithoutInputsBeforeGoingToSleep = new YoDouble("timeWithoutInputsBeforeGoingToSleep", registry);
      private final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
      private final AtomicReference<ToolboxState> newToolboxStateRequestedRef = new AtomicReference<>();
      private final YoEnum<ToolboxState> toolboxState = new YoEnum<>("toolboxState", registry, ToolboxState.class);

      public IKStreamingRTControllerThread(String robotName,
                                           ROS2NodeInterface ros2Node,
                                           CommandInputManager walkingInputManager,
                                           StatusMessageOutputManager walkingOutputManager,
                                           FullHumanoidRobotModelFactory fullRobotModelFactory,
                                           RobotCollisionModel collisionModel,
                                           KinematicsStreamingToolboxParameters parameters)
      {
         timeOfLastInput.set(Double.NEGATIVE_INFINITY);
         timeWithoutInputsBeforeGoingToSleep.set(parameters.getTimeThresholdForSleeping());

         ROS2Topic<?> inputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
         ROS2Topic<?> outputTopic = KinematicsStreamingToolboxModule.getOutputTopic(robotName);

         FullHumanoidRobotModel desiredFullRobotModel = fullRobotModelFactory.createFullRobotModel(false);
         this.commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
         this.statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());
         ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(inputTopic,
                                                                                                   commandInputManager,
                                                                                                   outputTopic,
                                                                                                   statusOutputManager,
                                                                                                   ros2Node);
         commandInputManager.registerHasReceivedInputListener(commandClass -> receivedInput.set(true));

         this.kinematicsStreamingToolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                                              statusOutputManager,
                                                                                              parameters,
                                                                                              desiredFullRobotModel,
                                                                                              fullRobotModelFactory,
                                                                                              yoGraphicsListRegistry,
                                                                                              registry);
         kinematicsStreamingToolboxController.setCollisionModel(collisionModel);
         kinematicsStreamingToolboxController.setStreamingMessagePublisher(walkingInputManager::submitMessage);
         kinematicsStreamingToolboxController.setTrajectoryMessagePublisher(walkingInputManager::submitMessage);
         walkingOutputManager.attachStatusMessageListener(RobotConfigurationData.class, kinematicsStreamingToolboxController::updateRobotConfigurationData);
         walkingOutputManager.attachStatusMessageListener(CapturabilityBasedStatus.class, kinematicsStreamingToolboxController::updateCapturabilityBasedStatus);

         ToolboxStateMessage message = new ToolboxStateMessage();
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, ToolboxStateMessage.class, inputTopic, s ->
         {
            s.takeNextData(message, null);
            newToolboxStateRequestedRef.set(ToolboxState.fromByte(message.getRequestedToolboxState()));
         });

         toolboxState.set(ToolboxState.SLEEP);
      }

      private long initialTime = -1L;

      @Override
      public void run()
      {
         ToolboxState newToolboxStateRequested = newToolboxStateRequestedRef.getAndSet(null);
         if (newToolboxStateRequested != null)
         {
            kinematicsStreamingToolboxController.notifyToolboxStateChange(newToolboxStateRequested);
            switch (newToolboxStateRequested)
            {
               case WAKE_UP:
                  receivedInput.set(true);
                  toolboxState.set(ToolboxState.WAKE_UP);
                  break;
               case REINITIALIZE:
                  kinematicsStreamingToolboxController.requestInitialize();
                  receivedInput.set(true);
                  break;
               default:
                  toolboxState.set(ToolboxState.SLEEP);
                  break;
            }
         }

         long currentMonotonicClockTime = System.nanoTime(); // FIXME ?
         if (initialTime < 0)
         {
            initialTime = currentMonotonicClockTime;
         }

         currentMonotonicClockTime -= initialTime;

         if (receivedInput.getAndSet(false))
         {
            timeOfLastInput.set(Conversions.nanosecondsToSeconds(currentMonotonicClockTime));
         }

         double timeSinceLastInput = Conversions.nanosecondsToSeconds(currentMonotonicClockTime) - timeOfLastInput.getValue();

         if (timeSinceLastInput > timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
         {
            toolboxState.set(ToolboxState.SLEEP);
            kinematicsStreamingToolboxController.notifyToolboxStateChange(ToolboxState.SLEEP);
         }

         if (toolboxState.getValue() == ToolboxState.WAKE_UP)
            kinematicsStreamingToolboxController.update();
      }

      @Override
      public YoRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public FullHumanoidRobotModel getFullRobotModel()
      {
         return kinematicsStreamingToolboxController.getDesiredFullRobotModel();
      }

      @Override
      public HumanoidRobotContextData getHumanoidRobotContextData()
      {
         return null;
      }

      public YoGraphicsListRegistry getSCS1YoGraphicsListRegistry()
      {
         return yoGraphicsListRegistry;
      }

      @Override
      public YoGraphicGroupDefinition getSCS2YoGraphics()
      {
         return null;
      }
   }
}
