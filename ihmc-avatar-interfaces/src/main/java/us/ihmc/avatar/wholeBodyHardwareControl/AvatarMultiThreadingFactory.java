package us.ihmc.avatar.wholeBodyHardwareControl;

import static us.ihmc.avatar.wholeBodyHardwareControl.AvatarMultiThreadingManager.runAll;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.*;

import org.apache.commons.math3.util.Precision;
import us.ihmc.affinity.Processor;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.IKStreamingRTPluginFactory;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommandBlenderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class is responsible for creating the estimator, controller, and optionally step generator and IK streaming modules.
 * It will turn each of those into runnable tasks, and potentially into standalone threads (realtime or non-realtime) depending on if the
 * desired threading setup is single-threaded or multi-threaded, and realtime or non-realtime (these settings can be configured using the
 * constructor arguments {@link #useMultiThreading} and {@link #useRealtimeThreads}). In addition, this class allows for the creation and
 * addition of custom runnable tasks, and it will turn those into standalone threads based on the aforementioned threading settings. Once
 * the desired task/threading setup is complete, call {@link #buildThreadsAndThreadingManager()} to finish task/thread creation and to
 * create an {@link AvatarMultiThreadingManager}, which handles the execution of all tasks and threads in a thread-safe manner.
 *
 * @author Stefan Fasano
 */
public class AvatarMultiThreadingFactory
{
   private static final double GRAVITY = -9.81;
   public static final boolean RUN_AUTO_DIAGNOSTIC = false;
   private static final int ROS2_PRIORITY = 25;

   private final YoRegistry rootRegistry;

   // Robot model
   private final DRCRobotModel masterRobotModel;
   private final FullHumanoidRobotModel masterFullRobotModel;
   private final HumanoidRobotContextData masterContext;
   private final HumanoidRobotContextDataFactory controllerContextFactory = new HumanoidRobotContextDataFactory();

   // Hardware communication API
   private final HardwareCommunicationInterface hardwareCommunicationInterface;

   // ROS stuff
   public final String IHMC_ROS_STATE_ESTIMATOR_NODE_NAME;
   public final String IHMC_ROS_CONTROLLER_NODE_NAME;
   public final String IHMC_ROS_IKSTREAMING_NODE_NAME;
   private final AsyncROS2Node estimatorAsyncROS2Node;
   private final AsyncROS2Node controllerAsyncROS2Node;
   private final PeriodicThreadSchedulerFactory ros2ThreadFactory;

   // Estimator
   private final AvatarEstimatorThread avatarEstimator;
   private final OptionalFactoryField<Map<HighLevelControllerName, StateEstimatorMode>> estimatorModeMapReference = new OptionalFactoryField<>("estimatorModeMapReference");
   private final List<Runnable> preEstimatorRunnables = new ArrayList<>();
   private final List<Runnable> postEstimatorRunnables = new ArrayList<>();

   // Controller
   private final HighLevelHumanoidControllerFactory avatarControllerFactory;
   private AvatarControllerThread avatarController;
   private final List<Runnable> preControllerRunnables = new ArrayList<>();
   private final List<Runnable> postControllerRunnables = new ArrayList<>();

   // Step Generator
   private final OptionalFactoryField<AvatarStepGeneratorThread> avatarStepGenerator = new OptionalFactoryField<>("AvatarStepGeneratorThread");

   // IK Streaming
   private final OptionalFactoryField<IKStreamingRTPluginFactory.IKStreamingRTThread> avatarIKStreaming = new OptionalFactoryField<>("AvatarIKStreamingThread");

   // List of all child threads and tasks
   private final List<Runnable> threads = new ArrayList<>();
   private final List<HumanoidRobotControlTask> tasks = new ArrayList<>();

   // Optional externally set master thread (for example EtherCAT thread)
   private final OptionalFactoryField<Runnable> externalMasterThread = new OptionalFactoryField<>("ExternallSetMasterThread");

   // Multi-threading manager
   private final RequiredFactoryField<AvatarMultiThreadingManager> threadingManager = new RequiredFactoryField<>("AvatarMultiThreadingManager");

   // Output processor
   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;

   // The remaining constructor arguments
   private final MonotonicTime period;
   private final double masterThreadDt;
   private final TimestampProvider monotonicTimeProvider;
   private final AvatarAffinityInterface affinity;
   private final boolean useRealtimeThreads;
   private final boolean useMultiThreading;
   private final YoVariableServer yoVariableServer;

   public AvatarMultiThreadingFactory(DRCRobotModel robotModel,
                                      FullHumanoidRobotModel fullRobotModel,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                      SensorReaderFactory sensorReaderFactory,
                                      HighLevelControllerStateFactory standPrepStateFactory,
                                      HighLevelControllerStateFactory freezeStateFactory,
                                      AvatarAffinityInterface affinity,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      MonotonicTime period,
                                      double masterThreadDt,
                                      TimestampProvider monotonicTimeProvider,
                                      YoRegistry registry,
                                      YoVariableServer yoVariableServer)
   {
      this.masterRobotModel = robotModel;
      this.masterFullRobotModel = fullRobotModel;
      this.period = period;
      this.masterThreadDt = masterThreadDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.lowLevelOutputProcessor = lowLevelOutputProcessor;
      this.affinity = affinity;
      this.useRealtimeThreads = useRealtimeThreads;
      this.useMultiThreading = useMultiThreading;
      this.rootRegistry = registry;
      this.yoVariableServer = yoVariableServer;

      masterContext = new HumanoidRobotContextData(masterFullRobotModel);

      // Estimator and controller ROS2 nodes
      IHMC_ROS_STATE_ESTIMATOR_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_ihmc_state_estimator";
      IHMC_ROS_CONTROLLER_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_" + HumanoidControllerAPI.HUMANOID_CONTROL_MODULE_NAME;
      IHMC_ROS_IKSTREAMING_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_ihmc_ikstreaming";

      if (useRealtimeThreads)
         ros2ThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(new PriorityParameters(ROS2_PRIORITY));
      else
         ros2ThreadFactory = new PeriodicNonRealtimeThreadSchedulerFactory();

      estimatorAsyncROS2Node = new AsyncROS2Node(IHMC_ROS_STATE_ESTIMATOR_NODE_NAME);
      controllerAsyncROS2Node = new AsyncROS2Node(IHMC_ROS_CONTROLLER_NODE_NAME);

      // Create estimator and estimator thread
      avatarEstimator = createAndAddEstimatorThread(robotModel, sensorReaderFactory);

      // Create high-level controller factory
      avatarControllerFactory = createHighLevelControllerFactory(robotModel,
                                                                 controllerAsyncROS2Node,
                                                                 lowLevelOutputProcessor,
                                                                 standPrepStateFactory,
                                                                 freezeStateFactory);
      avatarControllerFactory.setListenToHighLevelStatePackets(true);

      // Set the root registry as the YoVariableServer's main registry
      yoVariableServer.setMainRegistry(rootRegistry,
                                       masterFullRobotModel.getRootJoint().subtreeList(),
                                       new YoGraphicGroupDefinition(rootRegistry.getName()));

      // Add estimator thread registry directly to the YoVariableServer (since it is in a separate thread)
      yoVariableServer.addRegistry(avatarEstimator.getYoRegistry(), avatarEstimator.getSCS2YoGraphics());
   }

   public AvatarMultiThreadingManager buildThreadsAndThreadingManager()
   {
      // Create controller thread
      avatarController = createAndAddControllerThread();

      // Create threading manager
      threadingManager.set(new AvatarMultiThreadingManager(masterRobotModel.getSimpleRobotName().toLowerCase(),
                                                           masterContext,
                                                           hardwareCommunicationInterface,
                                                           lowLevelOutputProcessor,
                                                           threads,
                                                           tasks,
                                                           affinity,
                                                           masterThreadDt,
                                                           period,
                                                           monotonicTimeProvider,
                                                           useRealtimeThreads,
                                                           useMultiThreading,
                                                           externalMasterThread.hasValue() ? externalMasterThread.get() : null,
                                                           yoVariableServer,
                                                           rootRegistry));

      // Set up the block to prevent execution whenever there is no new state message.
      threadingManager.get().setBlockingProvider(() -> !hardwareCommunicationInterface.hasNewStateMessage());

      // Set up runnable to update master robot model with master context
      threadingManager.get().addPostMasterThreadRunnable(()->HumanoidRobotContextTools.updateRobot(masterFullRobotModel, masterContext.getProcessedJointData()));

      // Set up local logging
      LogTools.info("[Logging] Logging remote to logger server");

      return threadingManager.get();
   }

   /**
    * Creates the state estimator module and thread, and adds it to the multi-threaded setup
    */
   private AvatarEstimatorThread createAndAddEstimatorThread(DRCRobotModel robotModel, SensorReaderFactory sensorReaderFactory)
   {
      LogTools.info("The Squirrel estimates the number of acorns he needs for winter. Not many in Florida he thinks");
      HumanoidRobotContextDataFactory estimatorContextDataFactory = new HumanoidRobotContextDataFactory();

      AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
      avatarEstimatorThreadFactory.setROS2Info(estimatorAsyncROS2Node, robotModel.getSimpleRobotName());

      avatarEstimatorThreadFactory.configureWithWholeBodyControllerParameters(robotModel);
      avatarEstimatorThreadFactory.configureWithDRCRobotModel(robotModel);
      avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
      avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(estimatorContextDataFactory);
      avatarEstimatorThreadFactory.setGravity(GRAVITY);
      //      if (secondaryEstimatorFactory != null)
      //         avatarEstimatorThreadFactory.addSecondaryStateEstimatorFactory(secondaryEstimatorFactory);
      StateEstimatorController stateEstimator = avatarEstimatorThreadFactory.getMainStateEstimator();

      // Create state estimator from its factory
      AvatarEstimatorThread avatarEstimator = avatarEstimatorThreadFactory.createAvatarEstimatorThread();

      // Set up the task and thread for the state estimator
      setupEstimatorTaskAndThread(masterRobotModel, avatarEstimator, masterFullRobotModel, yoVariableServer);

      return avatarEstimator;
   }

   /**
    * Creates the high-level controller module and thread, and adds it to the multi-threaded setup
    */
   private AvatarControllerThread createAndAddControllerThread()
   {
      // Set up map that links high-level control states to estimator modes
      if (estimatorModeMapReference.hasValue())
      {
         avatarEstimator.setupHighLevelControllerCallback(avatarControllerFactory, estimatorModeMapReference.get());
      }
      else
      {
         HashMap<HighLevelControllerName, StateEstimatorMode> stateModeMap = new HashMap<>();
         Arrays.stream(HighLevelControllerName.values).forEach(name -> stateModeMap.put(name, StateEstimatorMode.FROZEN));
         stateModeMap.put(STAND_TRANSITION_STATE, StateEstimatorMode.NORMAL);
         stateModeMap.put(EXIT_WALKING, StateEstimatorMode.NORMAL);
         stateModeMap.put(WALKING, StateEstimatorMode.NORMAL);
         avatarEstimator.setupHighLevelControllerCallback(avatarControllerFactory, stateModeMap);
      }

      // Create the high-level controller
      AvatarControllerThread avatarController = new AvatarControllerThread(masterRobotModel.getSimpleRobotName().toLowerCase(),
                                                                                 masterRobotModel,
                                                                                 null,
                                                                                 masterRobotModel.getSensorInformation(),
                                                                                 avatarControllerFactory,
                                                                                 controllerContextFactory,
                                                                                 null,
                                                                                 controllerAsyncROS2Node,
                                                                                 GRAVITY,
                                                                                 false);

      // Add controller registry directly to the YoVariableServer (since it is in a separate thread)
      yoVariableServer.addRegistry(avatarController.getYoVariableRegistry(), avatarController.getSCS2YoGraphics());

      // Set up the task and thread for the controller
      setupControllerTaskAndThread(avatarController, masterFullRobotModel, yoVariableServer);

      return avatarController;
   }

   /**
    * An externally-facing API for creating the Step Generator module and thread, and for adding it to the multi-threaded setup
    */
   public AvatarStepGeneratorThread createAndAddStepGeneratorThread()
   {
      AvatarStepGeneratorThread stepGenerator = new AvatarStepGeneratorThread(controllerContextFactory,
                                                                              avatarControllerFactory.getStatusOutputManager(),
                                                                              avatarControllerFactory.getCommandInputManager(),
                                                                              masterRobotModel,
                                                                              null,
                                                                              controllerAsyncROS2Node);

      avatarStepGenerator.set(stepGenerator);

      yoVariableServer.addRegistry(avatarStepGenerator.get().getYoVariableRegistry(), avatarStepGenerator.get().getSCS2YoGraphics());

      setupStepGeneratorTaskAndThread(masterRobotModel,
                                      stepGenerator,
                                      yoVariableServer);

      return stepGenerator;
   }

   /**
    * An externally-facing API for creating the IK Streaming module and thread, and for adding it to the multi-threaded setup
    */
   public IKStreamingRTPluginFactory.IKStreamingRTThread createAndAddIKStreamingThread(KinematicsStreamingToolboxParameters ikStreamingParameters)
   {
      avatarIKStreaming.set(new IKStreamingRTPluginFactory().createRTThread(masterRobotModel.getSimpleRobotName(),
                                                                            estimatorAsyncROS2Node,
                                                                            avatarControllerFactory.getCommandInputManager(),
                                                                            avatarControllerFactory.getStatusOutputManager(), masterRobotModel,
                                                                            controllerContextFactory,
                                                                            masterRobotModel.getHumanoidRobotKinematicsCollisionModel(),
                                                                            ikStreamingParameters));

      if (yoVariableServer != null && avatarIKStreaming.get().isYoVariableServerEnabled())
         yoVariableServer.addRegistry(avatarIKStreaming.get().getYoVariableRegistry(), avatarIKStreaming.get().getSCS2YoGraphics());

      setupIKStreamingTaskAndThread(avatarIKStreaming.get(), yoVariableServer);

      return avatarIKStreaming.get();
   }

   /**
    * Sets up the actual thread and thread task for the state estimator module
    */
   private HumanoidRobotControlTask setupEstimatorTaskAndThread(DRCRobotModel robotModel,
                                                                AvatarEstimatorThread estimatorThread,
                                                                FullHumanoidRobotModel masterFullRobotModel,
                                                                YoVariableServer yoVariableServer)
   {
      // Set up Estimator Task
      int estimatorDivisor = (int) Math.round(robotModel.getEstimatorDT() / masterThreadDt);
      if (!Precision.equals(robotModel.getEstimatorDT() / masterThreadDt, estimatorDivisor))
         throw new RuntimeException("Estimator DT must be multiple of master thread DT.");

      EstimatorTask estimatorTask = new EstimatorTask(estimatorThread, estimatorDivisor, masterThreadDt, masterFullRobotModel);

      // Add post-estimator callback to update YoVariable server with estimator registry
      if (yoVariableServer != null)
         estimatorTask.addCallbackPostTask(() -> yoVariableServer.update(estimatorThread.getHumanoidRobotContextData().getTimestamp(),
                                                                         estimatorThread.getYoRegistry()));

      // Add pre-task callback to run all externally-set pre-estimator runnables
      estimatorTask.addCallbackPreTask(() -> runAll(preEstimatorRunnables));

      // Add post-estimator callback to update the thread frequency calculator and YoVariable (Hz)
      YoDouble estimatorThreadUpdateRate = new YoDouble("estimatorThreadUpdateRate", rootRegistry);
      FrequencyCalculator estimatorThreadFrequencyCalculator = new FrequencyCalculator(false);
      estimatorTask.addCallbackPostTask(()->
                                        {
                                           estimatorThreadFrequencyCalculator.ping();
                                           estimatorThreadUpdateRate.set(estimatorThreadFrequencyCalculator.getFrequency());
                                        });

      // Add post-task callback to hand sensor processor data to output processor in thread-safe manner
      estimatorTask.addCallbackPostTask(() ->
                                        {
                                           OneDoFJointReadOnly[] controlledJoints = estimatorThread.getFullRobotModel().getControllableOneDoFJoints();
                                           SensorProcessing sensorProcessing = estimatorThread.getSensorReader().getSensorProcessing();
                                           if (lowLevelOutputProcessor.getOffsetDesiredsByFilters())
                                           {
                                              for (OneDoFJointReadOnly controlledJoint : controlledJoints)
                                              {
                                                 lowLevelOutputProcessor.setJointPositionOffset(controlledJoint, sensorProcessing.getTotalPositionOffset(controlledJoint));
                                                 lowLevelOutputProcessor.setJointVelocityOffset(controlledJoint, sensorProcessing.getTotalVelocityOffset(controlledJoint));
                                              }
                                           }
                                        });

      // Add post-task callback to run all externally-set post-estimator runnables
      estimatorTask.addCallbackPostTask(() -> runAll(postEstimatorRunnables));

      // Add estimatorr startup callback to start spinning node
      estimatorTask.addRunnableOnStartup(() -> LogTools.info("Estimator node has started"));

      // Add estimator cleanup callback to stop spinning node
      estimatorTask.addRunnableOnCleanup(() -> LogTools.info("Estimator node has stopped spinning"));

      // Add estimator cleanup callback to destroy node
      estimatorTask.addRunnableOnCleanup(() ->
                                         {
                                            if (!estimatorAsyncROS2Node.isClosed())
                                               estimatorAsyncROS2Node.close();
                                            LogTools.info("Estimator node has been destroyed");
                                         });

      // Set up estimator realtime or non-realtime thread
      createAndAddThread(estimatorTask.getClass().getSimpleName(), estimatorTask, affinity.getEstimatorThreadProcessor(), affinity.getEstimatorThreadPriority());

      return estimatorTask;
   }

   /**
    * Sets up the actual thread and thread task for the high-level control module
    */
   private HumanoidRobotControlTask setupControllerTaskAndThread(AvatarControllerThread controllerThread,
                                                                 FullHumanoidRobotModel masterFullRobotModel,
                                                                 YoVariableServer yoVariableServer)
   {
      // Set up Controller Task
      ControllerTask controllerTask = new ControllerTask("Controller", controllerThread, masterThreadDt, masterFullRobotModel);

      // Add post-controller callback to update YoVariable server with controller registry
      if (yoVariableServer != null)
         controllerTask.addCallbackPostTask(() -> yoVariableServer.update(controllerThread.getHumanoidRobotContextData().getTimestamp(),
                                                                          controllerThread.getYoVariableRegistry()));

      // Add pre-task callback to run all externally-set pre-controller runnables
      controllerTask.addCallbackPreTask(() -> runAll(preControllerRunnables));

      // Add post-controller callback to interpolate desired setpoints
      controllerTask.addCallbackPostTask(lowLevelOutputProcessor::startDesiredsInterpolation);

      // Add post-controller callback to update the thread frequency calculator and YoVariable (Hz)
      YoDouble controllerThreadUpdateRate = new YoDouble("controllerThreadUpdateRate", rootRegistry);
      FrequencyCalculator controllerThreadFrequencyCalculator = new FrequencyCalculator(false);
      controllerTask.addCallbackPostTask(()->
                                         {
                                            controllerThreadFrequencyCalculator.ping();
                                            controllerThreadUpdateRate.set(controllerThreadFrequencyCalculator.getFrequency());
                                         });

      // Add pre-task callback to run all externally-set post-controller runnables
      controllerTask.addCallbackPostTask(() -> runAll(postControllerRunnables));

      // Add controller startup callback to start spinning node
      controllerTask.addRunnableOnStartup(() -> LogTools.info("Controller node has started"));

      // Add controller cleanup callback to stop spinning node
      controllerTask.addRunnableOnCleanup(() -> LogTools.info("Controller node has stopped spinning"));

      // Add controller cleanup callback to destroy node
      controllerTask.addRunnableOnCleanup(() ->
                                          {
                                             if (!controllerAsyncROS2Node.isClosed())
                                                controllerAsyncROS2Node.close();
                                             LogTools.info("Controller node has been destroyed");
                                          });

      // Set up and add controller realtime or non-realtime thread
      createAndAddThread(controllerTask.getClass().getSimpleName(), controllerTask, affinity.getControllerThreadProcessor(), affinity.getControllerThreadPriority());

      return controllerTask;
   }

   /**
    * Sets up the actual thread and thread task for the step generator module
    */
   private HumanoidRobotControlTask setupStepGeneratorTaskAndThread(DRCRobotModel robotModel,
                                                                    AvatarStepGeneratorThread stepGeneratorThread,
                                                                    YoVariableServer yoVariableServer)
   {
      // Set up Step Generator Task
      int stepGeneratorDivisor = (int) Math.round(robotModel.getStepGeneratorDT() / masterThreadDt);
      if (!Precision.equals(robotModel.getStepGeneratorDT() / masterThreadDt, stepGeneratorDivisor))
         throw new RuntimeException("Step generator DT must be multiple of master thread DT.");

      StepGeneratorTask stepGeneratorTask = new StepGeneratorTask("StepGenerator", stepGeneratorThread, stepGeneratorDivisor, masterThreadDt);

      // Add post-step generator callback to update YoVariable server with step generator registry
      if (yoVariableServer != null)
         stepGeneratorTask.addCallbackPostTask(() -> yoVariableServer.update(stepGeneratorThread.getHumanoidRobotContextData().getTimestamp(),
                                                                             stepGeneratorThread.getYoVariableRegistry()));

      // Add post-step generator callback to update the thread frequency calculator and YoVariable (Hz)
      YoDouble stepGeneratorThreadUpdateRate = new YoDouble("stepGeneratorThreadUpdateRate", rootRegistry);
      FrequencyCalculator stepGeneratorThreadFrequencyCalculator = new FrequencyCalculator(false);
      stepGeneratorTask.addCallbackPostTask(()->
                                            {
                                               stepGeneratorThreadFrequencyCalculator.ping();
                                               stepGeneratorThreadUpdateRate.set(stepGeneratorThreadFrequencyCalculator.getFrequency());
                                            });

      // Add callback on cleanup to close and destroy things
      stepGeneratorTask.addRunnableOnCleanup(avatarStepGenerator.get()::destroy);

      // Set up and add step generator realtime or non-realtime thread
      createAndAddThread(stepGeneratorTask.getClass().getSimpleName(), stepGeneratorTask, affinity.getStepGeneratorThreadProcessor(), affinity.getStepGeneratorThreadPriority());

      return stepGeneratorTask;
   }

   /**
    * Sets up the actual thread and thread task for the IK streaming module
    */
   private HumanoidRobotControlTask setupIKStreamingTaskAndThread(IKStreamingRTPluginFactory.IKStreamingRTThread ikStreamingThread,
                                                                  YoVariableServer yoVariableServer)
   {
      // Set up Step Generator Task
      IKStreamingRTPluginFactory.IKStreamingRTTask ikStreamingTask = IKStreamingRTPluginFactory.createIKStreamingRTTask(ikStreamingThread, masterThreadDt);

      // Add post-IK streaming callback to update YoVariable server with IK streaming registry
      if (yoVariableServer != null && ikStreamingThread.isYoVariableServerEnabled())
         ikStreamingTask.addCallbackPostTask(() -> yoVariableServer.update(ikStreamingThread.getHumanoidRobotContextData().getTimestamp(),
                                                                           ikStreamingThread.getYoVariableRegistry()));

      // Add post-IK streaming callback to update the thread frequency calculator and YoVariable (Hz)
      YoDouble ikStreamingThreadUpdateRate = new YoDouble("ikStreamingThreadUpdateRate", rootRegistry);
      FrequencyCalculator ikStreamingThreadFrequencyCalculator = new FrequencyCalculator(false);
      ikStreamingTask.addCallbackPostTask(() ->
                                          {
                                             ikStreamingThreadFrequencyCalculator.ping();
                                             ikStreamingThreadUpdateRate.set(ikStreamingThreadFrequencyCalculator.getFrequency());
                                          });

      // Set up and add IK streaming realtime or non-realtime thread
      createAndAddThread(ikStreamingTask.getClass().getSimpleName(), ikStreamingTask, affinity.getIKStreamingThreadProcessor(), affinity.getIKStreamingThreadPriority());

      return ikStreamingTask;
   }

   /**
    * Creates and sets up factory for high-level controller
    */
   private HighLevelHumanoidControllerFactory createHighLevelControllerFactory(DRCRobotModel robotModel,
                                                                               AsyncROS2Node ros2Node,
                                                                               AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                                                               HighLevelControllerStateFactory standPrepStateFactory,
                                                                               HighLevelControllerStateFactory freezeStateFactory)
   {
      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionalContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      CoPTrajectoryParameters copTrajectoryParameters = robotModel.getCoPTrajectoryParameters();

      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    feetForceSensorNames,
                                                                                                    wristForceSensorNames,
                                                                                                    highLevelControllerParameters,
                                                                                                    walkingControllerParameters,
                                                                                                    copTrajectoryParameters,
                                                                                                    robotModel.getSplitFractionCalculatorParameters());

      if (RUN_AUTO_DIAGNOSTIC)
      {
         // The automated diagnostic controller is running in NadiaSensorReader, no need for a high-level controller.
         controllerFactory.setInitialState(DO_NOTHING_BEHAVIOR);
         controllerFactory.useDefaultDoNothingControlState();
      }
      else
      {
         controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), ros2Node);

         // TODO Clean this config up
         // setup states
         controllerFactory.setInitialState(highLevelControllerParameters.getDefaultInitialControllerState());
         controllerFactory.addCustomControlState(standPrepStateFactory);
         controllerFactory.addCustomControlState(freezeStateFactory);
         controllerFactory.useDefaultStandTransitionControlState(STAND_PREP_STATE, WALKING);
         controllerFactory.useDefaultWalkingControlState();
         controllerFactory.useDefaultDoNothingControlState();
         controllerFactory.useDefaultFallingControlState();
         controllerFactory.useDefaultExitWalkingTransitionControlState(STAND_PREP_STATE);

         // setup transitions
         HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();

         controllerFactory.addRequestableTransition(STAND_PREP_STATE, STAND_TRANSITION_STATE);
         controllerFactory.addRequestableTransition(STAND_TRANSITION_STATE, STAND_PREP_STATE);
         controllerFactory.addRequestableTransition(FREEZE_STATE, STAND_PREP_STATE);
         controllerFactory.addRequestableTransition(WALKING, EXIT_WALKING);
         controllerFactory.addRequestableTransition(FALLING_STATE, STAND_PREP_STATE);

         // Always be able to request to go to freeze, falling, do nothing, and whatever the fallback state is
         for (HighLevelControllerName highLevelControllerName : HighLevelControllerName.values)
         {
            if (!highLevelControllerName.equals(DO_NOTHING_BEHAVIOR))
               controllerFactory.addRequestableTransition(highLevelControllerName, DO_NOTHING_BEHAVIOR);

            if (!highLevelControllerName.equals(FREEZE_STATE))
               controllerFactory.addRequestableTransition(highLevelControllerName, FREEZE_STATE);

            if (!highLevelControllerName.equals(FALLING_STATE))
               controllerFactory.addRequestableTransition(highLevelControllerName, FALLING_STATE);

            if (!highLevelControllerName.equals(fallbackControllerState))
               controllerFactory.addRequestableTransition(highLevelControllerName, fallbackControllerState);

            controllerFactory.addControllerFailureTransition(highLevelControllerName, fallbackControllerState);
         }

         controllerFactory.addFinishedTransition(STAND_TRANSITION_STATE, WALKING, false);
         controllerFactory.addFinishedTransition(EXIT_WALKING, FREEZE_STATE);

         controllerFactory.addCustomStateTransition(createStandTransitionState(STAND_TRANSITION_STATE,
                                                                               controllerFactory,
                                                                               !highLevelControllerParameters.automaticallyTransitionToWalkingWhenReady()));

         // Transition to DO_NOTHING when the robot is unservoed
         lowLevelOutputProcessor.addMasterGainListener(change ->
                                                       {
                                                          if (lowLevelOutputProcessor.getMasterGain().getValue() == 0.0)
                                                             controllerFactory.getRequestedControlStateEnum().set(DO_NOTHING_BEHAVIOR);
                                                       });

         // Add a way to control desired high-level controller state from hardware communication interface module
         hardwareCommunicationInterface.addRequestedHighLevelControlStateConsumer(highLevelControllerName -> controllerFactory.getRequestedControlStateEnum().set(highLevelControllerName));

         // Listener so that hardware communication interface knows the current high-level controller state
         controllerFactory.attachHighLevelStateChangedListener((from, to) -> hardwareCommunicationInterface.setCurrentHighLevelControllerState(to));
      }

      controllerFactory.setListenToHighLevelStatePackets(true);

      return controllerFactory;
   }

   public void removeControlState(HighLevelControllerName name)
   {
      avatarControllerFactory.removeControlState(name);
   }

   /**
    * The transition to the STAND_TRANSITION state will happen when: 1- the STAND_PREP is done, i.e.
    * the ramp up ratio is at 1, AND 2- the feet loaded transition is satisfied is requested via the
    * YoEnum.
    */
   private static ControllerStateTransitionFactory<HighLevelControllerName> createStandTransitionState(HighLevelControllerName transitionStateName,
                                                                                                       HighLevelHumanoidControllerFactory controllerFactory,
                                                                                                       boolean waitForRequestToTransition)
   {
      return new ControllerStateTransitionFactory<>()
      {
         @Override
         public HighLevelControllerName getStateToAttachEnum()
         {
            return STAND_PREP_STATE;
         }

         @Override
         public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> stateMap,
                                                                                    HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                                    YoRegistry parentRegistry)
         {
            HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactory.getHighLevelHumanoidControllerToolbox();
            double totalMass = controllerToolbox.getFullRobotModel().getTotalMass();
            double gravityZ = controllerToolbox.getGravityZ();
            YoEnum<HighLevelControllerName> requestedState = controllerFactory.getRequestedControlStateEnum();
            HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();

            StateTransitionCondition feetLoadedTransition = new FeetLoadedToWalkingStandTransition(transitionStateName,
                                                                                                   requestedState,
                                                                                                   waitForRequestToTransition,
                                                                                                   controllerToolbox.getFootSwitches(),
                                                                                                   highLevelControllerParameters.getControlDT(STAND_PREP_STATE),
                                                                                                   totalMass,
                                                                                                   gravityZ,
                                                                                                   highLevelControllerParameters,
                                                                                                   parentRegistry);

            State standPrepState = stateMap.get(STAND_PREP_STATE);

            StateTransitionCondition condition = new StateTransitionCondition()
            {
               @Override
               public boolean testCondition(double timeInCurrentState)
               {
                  if (!standPrepState.isDone(timeInCurrentState))
                     return false;

                  return feetLoadedTransition.testCondition(timeInCurrentState);
               }

               @Override
               public boolean performOnEntry()
               {
                  return true;
               }
            };

            return new StateTransition<HighLevelControllerName>(transitionStateName, condition);
         }
      };
   }

   /**
    * External API for adding a custom controller state to the high-level controller
    */
   public void addCustomControlState(HighLevelControllerStateFactory customControllerStateFactory)
   {
      avatarControllerFactory.addCustomControlState(customControllerStateFactory);
   }

   /**
    * External API for adding a requestable transition between two high-level control states
    */
   public void addRequestableTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      avatarControllerFactory.addRequestableTransition(currentControlStateEnum, nextControlStateEnum);
   }

   /**
    * External API for adding a done transition between two high-level control states
    */
   public void addFinishedTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      avatarControllerFactory.addFinishedTransition(currentControlStateEnum, nextControlStateEnum);
   }

   /**
    * External API for adding a done transition between two high-level control states
    */
   public void addFinishedTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum, boolean performNextStateOnEntry)
   {
      avatarControllerFactory.addFinishedTransition(currentControlStateEnum, nextControlStateEnum, performNextStateOnEntry);
   }

   /**
    * External API for adding a high-level controller state that is automatically transitioned into from a previous state (in this case the STAND_PREP
    * state), given logic that determines the trigger of that transition (in this case STAND_PREP is done, and feet are loaded)
    */
   public void addStandPrepStateTransition(HighLevelControllerName nextControlStateEnum)
   {
      avatarControllerFactory.addCustomStateTransition(createStandTransitionState(nextControlStateEnum, avatarControllerFactory, true));
   }

   /**
    * External API for adding a high-level control state that does a smooth blended transition between two other high-level control states
    */
   public void addSmoothTransitionState(String transitionName, HighLevelControllerName transitionStateEnum, HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      avatarControllerFactory.addCustomSmoothTransitionControlState(transitionName, transitionStateEnum, currentControlStateEnum, nextControlStateEnum);
   }

   /**
    * External API for adding a high-level control state that does a smooth blended transition between two other high-level control states
    */
   public void addSmoothTransitionState(String transitionName,
                                        HighLevelControllerName transitionStateEnum,
                                        HighLevelControllerName currentControlStateEnum,
                                        HighLevelControllerName nextControlStateEnum,
                                        CommandBlenderFactory commandBlenderFactory)
   {
      avatarControllerFactory.addCustomSmoothTransitionControlState(transitionName,
                                                                    transitionStateEnum,
                                                                    currentControlStateEnum,
                                                                    nextControlStateEnum,
                                                                    commandBlenderFactory);
   }

   /**
    * External API for providing a custom map between high-level control states and their respective state estimator mode
    */
   public void setHighLevelControllerCallbackForEstimator(Map<HighLevelControllerName, StateEstimatorMode> estimatorModeMap)
   {
      estimatorModeMapReference.set(estimatorModeMap);
   }

   /**
    * External API to allow addition of a new thread given a runnable, update rate, and priority/core (for realtime threads)
    * This will create a runnable task, add that to the list of tasks, and if needed, make/start a thread from that task
    */
   public HumanoidRobotControlTask createAndAddThread(String name, Runnable runnable, double threadDt, int core, int priority, YoRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry)
   {
      HumanoidRobotControlTask task = createAndAddTask(name, runnable, threadDt, registry, yoGraphicsRegistry);

      createAndAddThread(name, task, core, priority);

      return task;
   }

   /**
    * External API to allow addition of a new thread given a runnable and update rate
    * This will create a runnable task, add that to the list of tasks, and if needed, make/start a thread from that task
    */
   public HumanoidRobotControlTask createAndAddThread(String name, Runnable runnable, double threadDt, YoRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry)
   {
      HumanoidRobotControlTask task = createAndAddTask(name, runnable, threadDt, registry, yoGraphicsRegistry);

      createAndAddThread(name, task);

      return task;
   }

   /**
    * External API to allow addition of a new thread given a runnable task and priority/core (for realtime threads)
    * This will add the task to the list of tasks, and if needed, make/start a thread from that task
    */
   public void createAndAddThread(String name, HumanoidRobotControlTask task, int core, int priority)
   {
      addTask(task);

      // Set up thread
      if (useRealtimeThreads && useMultiThreading)
         createAndAddRealtimeThread(name, task, core, priority);
      else if (!useRealtimeThreads && useMultiThreading)
         createAndAddNonRealtimeThread(name, task);
   }

   /**
    * External API to allow addition of a new thread given a runnable task, thread processor, and thread core (for realtime threads)
    * This will add the task to the list of tasks, and if needed, make/start a thread from that task
    */
   public void createAndAddThread(String name, HumanoidRobotControlTask task, Processor threadProcessor, PriorityParameters threadPriority)
   {
      addTask(task);

      // Set up thread
      if (useRealtimeThreads && useMultiThreading)
         createAndAddRealtimeThread(name, task, threadProcessor, threadPriority);
      else if (!useRealtimeThreads && useMultiThreading)
         createAndAddNonRealtimeThread(name, task);
   }

   /**
    * External API to allow addition of a new thread given a runnable task
    * This will add the task to the list of tasks, and if needed, make/start a thread from that task
    */
   public void createAndAddThread(String name, HumanoidRobotControlTask task)
   {
      addTask(task);

      // Set up non-realtime thread
      if (!useRealtimeThreads && useMultiThreading)
         createAndAddNonRealtimeThread(name, task);
   }

   /**
    * External API to enable the creation of a runnable task given a name for that task, the runnable to be
    * run, and an update rate dt for that task
    */
   public HumanoidRobotControlTask createAndAddTask(String name, Runnable runnable, double threadDt, YoRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry)
   {
      // Calculate task divisor
      int divisor = (int) Math.round(threadDt / masterThreadDt);
      if (!Precision.equals(threadDt / masterThreadDt, divisor))
         throw new RuntimeException("Thread DT must be multiple of master thread DT.");

      // Set up thread timer to measure timing metrics of the thread
      ThreadTimer timer = new ThreadTimer(name, divisor * masterThreadDt, registry);

      HumanoidRobotControlTask task = new HumanoidRobotControlTask(divisor)
      {
         @Override
         protected boolean initialize()
         {
            timer.reset();
            return super.initialize();
         }

         @Override
         protected void execute()
         {
            // Start thread timer
            timer.start();

            // Execute pre-task runnables
            runAll(preTaskCallbacks);

            // Execute main loop of thread
            runnable.run();

            // Execute post-task runnables
            runAll(postTaskCallbacks);

            // Stop thread timer
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

         }
      };

      // Add this thread's YoRegistry to the YoVariable server
      yoVariableServer.addRegistry(registry);

      // Add post-thread callback to update YoVariable server with thread registry
      task.addCallbackPostTask(() -> yoVariableServer.update(RealtimeThread.getCurrentMonotonicClockTime(),
                                                             registry));

      // Add post-thread callback to update the thread frequency calculator and YoVariable (Hz)
      YoDouble threadUpdateRate = new YoDouble(name + "ThreadUpdateRate", rootRegistry);
      FrequencyCalculator threadFrequencyCalculator = new FrequencyCalculator(false);
      task.addCallbackPostTask(() ->
                               {
                                  threadFrequencyCalculator.ping();
                                  threadUpdateRate.set(threadFrequencyCalculator.getFrequency());
                               });

      // Add task to list of all tasks so it can be handled by barrier scheduler
      addTask(task);

      return task;
   }

   /**
    * Creates realtime thread, starts that thread, and adds that thread to the list of all threads
    */
   private RealtimeThread createAndAddRealtimeThread(String name, Runnable runnable, int core, int priority)
   {
      Processor threadProcessor = affinity.getSocket().getCore(affinity.checkCoreIsValid(core)).getDefaultProcessor();
      PriorityParameters threadPriority = new PriorityParameters(priority);

      return createAndAddRealtimeThread(name, runnable, threadProcessor, threadPriority);
   }

   /**
    * Creates realtime thread, starts that thread, and adds that thread to the list of all threads
    */
   private RealtimeThread createAndAddRealtimeThread(String name, Runnable runnable, Processor threadProcessor, PriorityParameters threadPriority)
   {
      RealtimeThread realtimeThread = new RealtimeThread(threadPriority, runnable, name + "Thread");
      realtimeThread.setAffinity(threadProcessor);
      realtimeThread.start();

      addThread(realtimeThread);
      return realtimeThread;
   }

   /**
    * Creates non-realtime thread, starts that thread, and adds that thread to the list of all threads
    */
   private Thread createAndAddNonRealtimeThread(String name, Runnable runnable)
   {
      Thread nonRealtimeThread = new Thread(runnable, name + "Thread");
      nonRealtimeThread.start();

      addThread(nonRealtimeThread);
      return nonRealtimeThread;
   }

   /**
    * For adding a new runnable task.
    * IMPORTANT: this is required for any updatable that needs to be run, either in a sequential single-threaded format, or
    * in a multi-threaded format. This is the list that is given to the task/runnable/thread scheduler, which in turn determines
    * and dictates the execution of everything in this list
    */
   public void addTask(HumanoidRobotControlTask task)
   {
      if (!tasks.contains(task))
         tasks.add(task);
   }

   /**
    * TODO eventually we get rid of this
    */
   public void addThread(Runnable thread)
   {
      threads.add(thread);
   }

   public void setExternalMasterThread(RealtimeThread masterThread)
   {
      externalMasterThread.set(masterThread);
   }

   public void setExternalMasterThread(RepeatingTaskThread masterThread)
   {
      externalMasterThread.set(masterThread);
   }

   public HighLevelHumanoidControllerFactory getControllerFactory()
   {
      return avatarControllerFactory;
   }

   public YoRegistry getEstimatorRegistry()
   {
      return avatarEstimator.getYoRegistry();
   }

   public YoGraphicGroupDefinition getEstimatorYoGraphics()
   {
      return avatarEstimator.getSCS2YoGraphics();
   }

   public AsyncROS2Node getEstimatorROS2Node()
   {
      return estimatorAsyncROS2Node;
   }

   public FullHumanoidRobotModel getEstimatorFullRobotModel()
   {
      return avatarEstimator.getFullRobotModel();
   }

   public void addPreEstimatorThreadRunnable(Runnable runnable)
   {
      preEstimatorRunnables.add(runnable);
   }

   public void addPostEstimatorThreadRunnable(Runnable runnable)
   {
      postEstimatorRunnables.add(runnable);
   }

   public void addPreControllerThreadRunnable(Runnable runnable)
   {
      preControllerRunnables.add(runnable);
   }

   public void addPostControllerThreadRunnable(Runnable runnable)
   {
      postControllerRunnables.add(runnable);
   }
}
