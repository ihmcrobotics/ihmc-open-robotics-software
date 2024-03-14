package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.AvatarControllerThreadInterface;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.Arrays;
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
                                                           HumanoidRobotContextDataFactory contextDataFactory,
                                                           RobotCollisionModel collisionModel)
   {
      if (ikStreamingRTController == null)
         ikStreamingRTController = new IKStreamingRTControllerThread(robotName,
                                                                     ros2Node,
                                                                     walkingInputManager,
                                                                     walkingOutputManager,
                                                                     fullRobotModelFactory,
                                                                     contextDataFactory,
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
      private final CrossRobotCommandResolver controllerResolver;
      private final IKStreamingRTControllerThread ikStreamingThread;

      private final long divisor;
      private final ThreadTimer timer;
      private final YoLong ticksBehindScheduled;

      private final List<Runnable> postControllerCallbacks = new ArrayList<>();
      private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

      public IKStreamingRTTask(String prefix,
                               IKStreamingRTControllerThread ikStreamingThread,
                               long divisor,
                               double schedulerDt,
                               FullHumanoidRobotModel masterFullRobotModel)
      {
         super(divisor);
         this.divisor = divisor;
         this.ikStreamingThread = ikStreamingThread;

         controllerResolver = new CrossRobotCommandResolver(ikStreamingThread.getFullRobotModel());

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

      @Override
      protected void execute()
      {
         timer.start();
         ticksBehindScheduled.set(ikStreamingThread.getHumanoidRobotContextData().getSchedulerTick() - timer.getTickCount() * divisor);
         ikStreamingThread.run();
         runAll(postControllerCallbacks);
         timer.stop();
      }

      @Override
      protected void updateMasterContext(HumanoidRobotContextData masterContext)
      {
         runAll(schedulerThreadRunnables);
      }

      @Override
      protected void updateLocalContext(HumanoidRobotContextData masterContext)
      {
         controllerResolver.resolveHumanoidRobotContextDataScheduler(masterContext, ikStreamingThread.getHumanoidRobotContextData());
         controllerResolver.resolveHumanoidRobotContextDataEstimator(masterContext, ikStreamingThread.getHumanoidRobotContextData());
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
      private final HumanoidRobotContextData humanoidRobotContextData;

      public IKStreamingRTControllerThread(String robotName,
                                           ROS2NodeInterface ros2Node,
                                           CommandInputManager walkingInputManager,
                                           StatusMessageOutputManager walkingOutputManager,
                                           FullHumanoidRobotModelFactory fullRobotModelFactory,
                                           HumanoidRobotContextDataFactory contextDataFactory,
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
         walkingOutputManager.attachStatusMessageListener(CapturabilityBasedStatus.class, kinematicsStreamingToolboxController::updateCapturabilityBasedStatus);

         ToolboxStateMessage message = new ToolboxStateMessage();
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, ToolboxStateMessage.class, inputTopic, s ->
         {
            s.takeNextData(message, null);
            newToolboxStateRequestedRef.set(ToolboxState.fromByte(message.getRequestedToolboxState()));
         });

         toolboxState.set(ToolboxState.SLEEP);

         HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(desiredFullRobotModel.getOneDoFJoints().length);
         ForceSensorDataHolder forceSensorDataHolderForController = new ForceSensorDataHolder(Arrays.asList(desiredFullRobotModel.getForceSensorDefinitions()));
         CenterOfMassDataHolder centerOfMassDataHolderForController = new CenterOfMassDataHolder();
         CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator = new CenterOfPressureDataHolder(desiredFullRobotModel);
         LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(desiredFullRobotModel.getControllableOneDoFJoints());
         RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
         contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForController);
         contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForController);
         contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForEstimator);
         contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
         contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
         contextDataFactory.setProcessedJointData(processedJointData);
         contextDataFactory.setSensorDataContext(new SensorDataContext(desiredFullRobotModel));
         humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

         kinematicsStreamingToolboxController.setRobotStateUpdater((rootJoint, oneDoFJoints) ->
                                                                   {
                                                                      if (!humanoidRobotContextData.getEstimatorRan())
                                                                         return false;
                                                                      HumanoidRobotContextTools.updateRobot(desiredFullRobotModel,
                                                                                                            humanoidRobotContextData.getProcessedJointData());
                                                                      return true;
                                                                   });
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

         if (toolboxState.getValue() == ToolboxState.WAKE_UP)
         {
            if (timeSinceLastInput > timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
            {
               toolboxState.set(ToolboxState.SLEEP);
               kinematicsStreamingToolboxController.notifyToolboxStateChange(ToolboxState.SLEEP);
            }

            kinematicsStreamingToolboxController.update();
         }
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
         return humanoidRobotContextData;
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
