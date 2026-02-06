package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.AvatarEstimatorThreadFactory;
import us.ihmc.avatar.AvatarStepGeneratorThread;
import us.ihmc.avatar.HumanoidSteppingPluginEnvironmentalConstraints;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.IKStreamingRTPluginFactory;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommandBlenderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.JoystickBasedSteppingPluginFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.log.LogTools;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.RealtimeROS2Node;
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
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.*;

/**
 * This class is responsible for creating the estimator, controller, and step generator
 * threads, along with the multi-threading manager that handles the execution of those threads.
 *
 * @author Stefan Fasano
 */
public class AvatarMultiThreadingFactory
{
   private static final double GRAVITY = -9.81;
   public static final boolean RUN_AUTO_DIAGNOSTIC = false;
   private static final int ROS2_PRIORITY = 25;
   private static final int JVM_STATISTICS_PRIORITY = 5;


   private final YoRegistry rootRegistry;

   // Robot model
   private final DRCRobotModel robotModel;
   private final HumanoidRobotContextDataFactory controllerContextFactory = new HumanoidRobotContextDataFactory();

   // Hardware communication API
   private final HardwareCommunicationInterface hardwareCommunicationInterface;

   // ROS stuff
   public final String IHMC_ROS_STATE_ESTIMATOR_NODE_NAME;
   public final String IHMC_ROS_CONTROLLER_NODE_NAME;
   public final String IHMC_ROS_IKSTREAMING_NODE_NAME;
   private final RealtimeROS2Node estimatorRealtimeROS2Node;
   private final RealtimeROS2Node controllerRealtimeROS2Node;
   private final PeriodicThreadSchedulerFactory ros2ThreadFactory;

   // The thread factories
   private final AvatarEstimatorThreadFactory estimatorThreadFactory;
   private final HighLevelHumanoidControllerFactory controllerFactory;

   // The threads
   private final RequiredFactoryField<AvatarEstimatorThread> estimatorThread = new RequiredFactoryField<>("AvatarEstimatorThread");
   private final RequiredFactoryField<AvatarControllerThread> controllerThread = new RequiredFactoryField<>("AvatarControllerThread");
   private final OptionalFactoryField<AvatarStepGeneratorThread> stepGeneratorThread = new OptionalFactoryField<>("AvatarStepGeneratorThread");
   private final OptionalFactoryField<Map<HighLevelControllerName, StateEstimatorMode>> estimatorModeMapReference = new OptionalFactoryField<>("estimatorModeMapReference");
   private final OptionalFactoryField<IKStreamingRTPluginFactory.IKStreamingRTThread> ikStreamingThread = new OptionalFactoryField<>("AvatarIKStreamingThread");

   // Multi-threading manager
   private final RequiredFactoryField<AvatarMultiThreadingManager> threadingManager = new RequiredFactoryField<>("AvatarMultiThreadingManager");

   // Output processor
   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;

   // The remaining constructor arguments
   private final MonotonicTime period;
   private final double masterThreadDt;
   private final TimestampProvider monotonicTimeProvider;
   private final AvatarAffinityInterface affinity;
   private final boolean createStepGeneratorThread;
   private final boolean useRealtimeThreads;
   private final boolean useMultiThreading;
   private final boolean useLocalLogging;
   private final YoVariableServer yoVariableServer;

   public AvatarMultiThreadingFactory(DRCRobotModel robotModel,
                                      FullHumanoidRobotModel fullRobotModel,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      AvatarLowLevelOutputProcessor lowLevelOutputProcessor,
                                      SensorReaderFactory sensorReaderFactory,
                                      HighLevelControllerStateFactory standPrepStateFactory,
                                      HighLevelControllerStateFactory freezeStateFactory,
                                      AvatarAffinityInterface affinity,
                                      boolean createStepGeneratorThread,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      boolean useLocalLogging,
                                      MonotonicTime period,
                                      double masterThreadDt,
                                      TimestampProvider monotonicTimeProvider,
                                      YoRegistry registry,
                                      YoVariableServer yoVariableServer)
   {
      this.robotModel = robotModel;
      this.period = period;
      this.masterThreadDt = masterThreadDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.lowLevelOutputProcessor = lowLevelOutputProcessor;
      this.affinity = affinity;
      this.createStepGeneratorThread = createStepGeneratorThread;
      this.useRealtimeThreads = useRealtimeThreads;
      this.useMultiThreading = useMultiThreading;
      this.useLocalLogging = useLocalLogging;
      this.rootRegistry = registry;
      this.yoVariableServer = yoVariableServer;

      // Estimator and controller ROS2 nodes
      IHMC_ROS_STATE_ESTIMATOR_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_ihmc_state_estimator";
      IHMC_ROS_CONTROLLER_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_" + HumanoidControllerAPI.HUMANOID_CONTROL_MODULE_NAME;
      IHMC_ROS_IKSTREAMING_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_ihmc_ikstreaming";

      if (useRealtimeThreads)
         ros2ThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(new PriorityParameters(ROS2_PRIORITY));
      else
         ros2ThreadFactory = new PeriodicNonRealtimeThreadSchedulerFactory();

      estimatorRealtimeROS2Node = new ROS2NodeBuilder().buildRealtime(IHMC_ROS_STATE_ESTIMATOR_NODE_NAME, ros2ThreadFactory);
      controllerRealtimeROS2Node = new ROS2NodeBuilder().buildRealtime(IHMC_ROS_CONTROLLER_NODE_NAME, ros2ThreadFactory);

      // Setup state estimator factory
      estimatorThreadFactory = createStateEstimatorFactory(robotModel, fullRobotModel, sensorReaderFactory);

      // Setup state controller factory
      controllerFactory = createHighLevelControllerFactory(robotModel,
                                                           controllerRealtimeROS2Node,
                                                           lowLevelOutputProcessor,
                                                           standPrepStateFactory,
                                                           freezeStateFactory);
   }

   public AvatarMultiThreadingManager buildThreadsAndThreadingManager()
   {
      // Create estimator thread
      estimatorThread.set(estimatorThreadFactory.createAvatarEstimatorThread());

      // Hand the communication module the sensor processor
      SensorProcessing sensorProcessing = estimatorThread.get().getSensorReader().getSensorProcessing();
      hardwareCommunicationInterface.setSensorProcessing(sensorProcessing);

      // Create controller thread
      if (estimatorModeMapReference.hasValue())
      {
         estimatorThread.get().setupHighLevelControllerCallback(controllerFactory, estimatorModeMapReference.get());
      }
      else
      {
         HashMap<HighLevelControllerName, StateEstimatorMode> stateModeMap = new HashMap<>();
         Arrays.stream(HighLevelControllerName.values).forEach(name -> stateModeMap.put(name, StateEstimatorMode.FROZEN));
         stateModeMap.put(STAND_TRANSITION_STATE, StateEstimatorMode.NORMAL);
         stateModeMap.put(EXIT_WALKING, StateEstimatorMode.NORMAL);
         stateModeMap.put(WALKING, StateEstimatorMode.NORMAL);
         estimatorThread.get().setupHighLevelControllerCallback(controllerFactory, stateModeMap);
      }

      controllerThread.set(new AvatarControllerThread(robotModel.getSimpleRobotName().toLowerCase(),
                                                      robotModel,
                                                      null,
                                                      robotModel.getSensorInformation(),
                                                      controllerFactory,
                                                      controllerContextFactory,
                                                      null,
                                                      controllerRealtimeROS2Node,
                                                      GRAVITY,
                                                      false));

      // Create step generator thread
      if (createStepGeneratorThread)
         stepGeneratorThread.set(createStepGeneratorThread(robotModel, controllerThread.get(), controllerContextFactory, controllerFactory));

      // Add estimator thread registry as child to the root registry (since estimator thread is essentially our master thread)
      rootRegistry.addChild(estimatorThread.get().getYoRegistry());

      // Set the root registry as the YoVariableServer's main registry
      yoVariableServer.setMainRegistry(rootRegistry,
                                       estimatorThread.get().getFullRobotModel().getRootJoint().subtreeList(),
                                       estimatorThread.get().getSCS2YoGraphics());

      // Add controller thread registry directly to the YoVariableServer (since it is in a separate thread)
      yoVariableServer.addRegistry(controllerThread.get().getYoVariableRegistry(), controllerThread.get().getSCS2YoGraphics());

      // Add step generator thread registry directly to the YoVariableServer (since it is in a separate thread)
      if (createStepGeneratorThread)
         yoVariableServer.addRegistry(stepGeneratorThread.get().getYoVariableRegistry(), stepGeneratorThread.get().getSCS2YoGraphics());

      if (ikStreamingThread.hasValue())
         yoVariableServer.addRegistry(ikStreamingThread.get().getYoVariableRegistry(), ikStreamingThread.get().getSCS2YoGraphics());

      // Setup JVM statistics
      PeriodicThreadSchedulerFactory jvmSchedulerFactory;
      if (useRealtimeThreads)
         jvmSchedulerFactory = new PeriodicRealtimeThreadSchedulerFactory(new PriorityParameters(JVM_STATISTICS_PRIORITY));
      else
         jvmSchedulerFactory = new PeriodicNonRealtimeThreadSchedulerFactory();

      JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer, jvmSchedulerFactory);
      jvmStatisticsGenerator.addVariablesToStatisticsGenerator(yoVariableServer);

      // Create threading manager
      threadingManager.set(new AvatarMultiThreadingManager(robotModel.getSimpleRobotName().toLowerCase(),
                                                           robotModel,
                                                           estimatorRealtimeROS2Node,
                                                           controllerRealtimeROS2Node,
                                                           estimatorThread.get().getHumanoidRobotContextData(),
                                                           estimatorThread.get().getFullRobotModel(),
                                                           hardwareCommunicationInterface,
                                                           lowLevelOutputProcessor,
                                                           estimatorThread.get(),
                                                           controllerThread.get(),
                                                           stepGeneratorThread.hasValue() ? stepGeneratorThread.get() : null,
                                                           ikStreamingThread.hasValue() ? ikStreamingThread.get() : null,
                                                           affinity,
                                                           masterThreadDt,
                                                           period,
                                                           monotonicTimeProvider,
                                                           useRealtimeThreads,
                                                           useMultiThreading,
                                                           yoVariableServer,
                                                           jvmStatisticsGenerator,
                                                           rootRegistry));

      // Set up the block to prevent execution whenever there is no new state message.
      threadingManager.get().setBlockingProvider(() -> !hardwareCommunicationInterface.hasNewStateMessage());

      if (useLocalLogging)
      {
         // Setup logger
         ArrayList<RegistrySendBufferBuilder> builders = new ArrayList<>();
         builders.add(new RegistrySendBufferBuilder(rootRegistry,
                                                    estimatorThread.get().getFullRobotModel().getRootJoint().subtreeList(),
                                                    estimatorThread.get().getSCS2YoGraphics()));
         builders.add(new RegistrySendBufferBuilder(controllerThread.get().getYoVariableRegistry(),
                                                    controllerThread.get().getSCS2YoGraphics()));
         if (stepGeneratorThread.hasValue())
         {
            builders.add(new RegistrySendBufferBuilder(stepGeneratorThread.get().getYoVariableRegistry(),
                                                       stepGeneratorThread.get().getSCS2YoGraphics()));
         }
         if (ikStreamingThread.hasValue())
         {
            builders.add(new RegistrySendBufferBuilder(ikStreamingThread.get().getYoVariableRegistry(),
                                                       ikStreamingThread.get().getSCS2YoGraphics()));
         }

         // FIXME add this back when a release of the logger is done.
//         builders.add(new RegistrySendBufferBuilder(jvmStatisticsGenerator.getYoRegistry(), null));

         // Logging locally on the robot
         IntraprocessYoVariableLogger intraprocessYoVariableLogger = new IntraprocessYoVariableLogger(builders,
                                                                         robotModel.getEstimatorDT(),
                                                                         robotModel.getSimpleRobotName().toLowerCase() + getClass().getSimpleName(), robotModel.getLogModelProvider());

         if (intraprocessYoVariableLogger.create())
         {
            LogTools.info("[Logging] Logging locally to disk");

            threadingManager.get()
                            .addPostEstimatorThreadRunnable(() -> intraprocessYoVariableLogger.update(estimatorThread.get()
                                                                                                                     .getHumanoidRobotContextData()
                                                                                                                     .getTimestamp()));
         }
         else
         {
            LogTools.error("[Logging] Unable to log locally to disk");
         }

         LogTools.info("[Logging] Logging locally to disk");
      }
      else
      {
         LogTools.info("[Logging] Logging remote to logger server");
      }

      return threadingManager.get();
   }

   /**
    * Create Estimator Factory
    */
   private AvatarEstimatorThreadFactory createStateEstimatorFactory(DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, SensorReaderFactory sensorReaderFactory)
   {
      LogTools.info("The Squirrel estimates the number of acorns he needs for winter. Not many in Florida he thinks");
      HumanoidRobotContextDataFactory estimatorContextDataFactory = new HumanoidRobotContextDataFactory();

      AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
      avatarEstimatorThreadFactory.setROS2Info(estimatorRealtimeROS2Node, robotModel.getSimpleRobotName());

      avatarEstimatorThreadFactory.configureWithWholeBodyControllerParameters(robotModel);
      avatarEstimatorThreadFactory.setEstimatorFullRobotModel(fullRobotModel);
      avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
      avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(estimatorContextDataFactory);
      avatarEstimatorThreadFactory.setGravity(GRAVITY);
      //      if (secondaryEstimatorFactory != null)
      //         avatarEstimatorThreadFactory.addSecondaryStateEstimatorFactory(secondaryEstimatorFactory);
      StateEstimatorController stateEstimator = avatarEstimatorThreadFactory.getMainStateEstimator();

      return avatarEstimatorThreadFactory;
   }

   /**
    * Create High Level Controller Factory
    */
   // Adding quicksterWalking parameters
   private HighLevelHumanoidControllerFactory createHighLevelControllerFactory(DRCRobotModel robotModel,
                                                                               RealtimeROS2Node ros2Node,
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
         controllerFactory.useDefaultExitWalkingTransitionControlState(STAND_PREP_STATE);

         // setup transitions
         HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();

         controllerFactory.addRequestableTransition(STAND_PREP_STATE, STAND_TRANSITION_STATE);
         controllerFactory.addRequestableTransition(STAND_TRANSITION_STATE, STAND_PREP_STATE);
         controllerFactory.addRequestableTransition(FREEZE_STATE, STAND_PREP_STATE);
         controllerFactory.addRequestableTransition(WALKING, EXIT_WALKING);

         // Always be able to request to go to freeze, since that's often a good failure state. Also add this as a failure transition for all states
         for (HighLevelControllerName highLevelControllerName : HighLevelControllerName.values)
         {
            if (highLevelControllerName == FREEZE_STATE)
               continue;

            controllerFactory.addRequestableTransition(highLevelControllerName, FREEZE_STATE);
            controllerFactory.addControllerFailureTransition(highLevelControllerName, fallbackControllerState);
         }

         for (HighLevelControllerName highLevelControllerName : HighLevelControllerName.values)
         {
            if (highLevelControllerName == DO_NOTHING_BEHAVIOR)
               continue;

            controllerFactory.addRequestableTransition(highLevelControllerName, DO_NOTHING_BEHAVIOR);
         }

         controllerFactory.addFinishedTransition(STAND_TRANSITION_STATE, WALKING, false);
         controllerFactory.addFinishedTransition(EXIT_WALKING, FREEZE_STATE);

         controllerFactory.addCustomStateTransition(createStandTransitionState(STAND_TRANSITION_STATE,
                                                                               controllerFactory,
                                                                               !highLevelControllerParameters.automaticallyTransitionToWalkingWhenReady()));

         // Transition to DO_NOTHING in the event of a fault
         hardwareCommunicationInterface.addFaultListener(change ->
                                                         {
                                                            if (hardwareCommunicationInterface.hasRobotFaulted())
                                                               controllerFactory.getRequestedControlStateEnum().set(DO_NOTHING_BEHAVIOR);
                                                         });
         // Transition to DO_NOTHING when the robot is unservoed
         lowLevelOutputProcessor.addMasterGainListener(change ->
                                                       {
                                                          if (lowLevelOutputProcessor.getMasterGain().getValue() == 0.0)
                                                             controllerFactory.getRequestedControlStateEnum().set(DO_NOTHING_BEHAVIOR);
                                                       });
      }

      controllerFactory.setListenToHighLevelStatePackets(true);

      return controllerFactory;
   }

   /**
    * Create Step Generator Thread
    */
   private AvatarStepGeneratorThread createStepGeneratorThread(DRCRobotModel robotModel,
                                                               AvatarControllerThread controllerThread,
                                                               HumanoidRobotContextDataFactory controllerContextFactory,
                                                               HighLevelHumanoidControllerFactory controllerFactory)
   {
      AvatarStepGeneratorThread stepGeneratorThread = null;

      LogTools.info("create step generator = " + createStepGeneratorThread);

      HumanoidSteppingPluginEnvironmentalConstraints environmentalConstraints = new HumanoidSteppingPluginEnvironmentalConstraints(robotModel.getContactPointParameters(),
                                                                                                                                   robotModel.getWalkingControllerParameters().getSteppingParametersForStepGeneration());
      controllerFactory.setListenToHighLevelStatePackets(true);

      JoystickBasedSteppingPluginFactory pluginFactory = new JoystickBasedSteppingPluginFactory();

      if (createStepGeneratorThread)
      {
         stepGeneratorThread = new AvatarStepGeneratorThread(pluginFactory,
                                                             controllerContextFactory,
                                                             controllerFactory.getStatusOutputManager(),
                                                             controllerFactory.getCommandInputManager(),
                                                             robotModel,
                                                             environmentalConstraints,
                                                             controllerRealtimeROS2Node);
      }
      else
      {
         // sets up the environmental constraint manager as a height map consumer in the input manager
         pluginFactory.addHeightMapCommandConsumer(environmentalConstraints);
         // Adds functions that adjust the footholds based on the environment.
         pluginFactory.setFootStepAdjustment(environmentalConstraints.getFootstepAdjustment());
         // Adds checkers for footholds based on the environment
         for (FootstepValidityIndicator footstepValidityIndicator : environmentalConstraints.getFootstepValidityIndicators())
            pluginFactory.addFootstepValidityIndicator(footstepValidityIndicator);
         // clear the environment at the beginning of every update
         pluginFactory.addUpdatable(environmentalConstraints);

         // create the callback listeners for the planar regions in the stepping plugin
         pluginFactory.createStepGeneratorNetworkSubscriber(robotModel.getSimpleRobotName().toLowerCase(), controllerRealtimeROS2Node);

         controllerFactory.addControllerPlugin(pluginFactory);

         // otherwise this would go into the step generator registry
         controllerThread.getYoVariableRegistry().addChild(environmentalConstraints.getRegistry());
      }

      return stepGeneratorThread;
   }

   public void addIKStreamingThread(KinematicsStreamingToolboxParameters ikStreamingParameters)
   {
      ikStreamingThread.set(new IKStreamingRTPluginFactory().createRTThread(robotModel.getSimpleRobotName(),
                                                                            estimatorRealtimeROS2Node,
                                                                            controllerFactory.getCommandInputManager(),
                                                                            controllerFactory.getStatusOutputManager(),
                                                                            robotModel,
                                                                            controllerContextFactory,
                                                                            robotModel.getHumanoidRobotKinematicsCollisionModel(),
                                                                            ikStreamingParameters));
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

   public void addCustomControlState(HighLevelControllerStateFactory customControllerStateFactory)
   {
      controllerFactory.addCustomControlState(customControllerStateFactory);
   }

   public void addRequestableTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      controllerFactory.addRequestableTransition(currentControlStateEnum, nextControlStateEnum);
   }

   public void addFinishedTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      controllerFactory.addFinishedTransition(currentControlStateEnum, nextControlStateEnum);
   }

   public void addStandPrepStateTransition(HighLevelControllerName nextControlStateEnum)
   {
      controllerFactory.addCustomStateTransition(createStandTransitionState(nextControlStateEnum, controllerFactory, true));
   }

   public void addFinishedTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum, boolean performNextStateOnEntry)
   {
      controllerFactory.addFinishedTransition(currentControlStateEnum, nextControlStateEnum, performNextStateOnEntry);
   }

   public void addSmoothTransitionState(String transitionName, HighLevelControllerName transitionStateEnum, HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      controllerFactory.addCustomSmoothTransitionControlState(transitionName, transitionStateEnum, currentControlStateEnum, nextControlStateEnum);
   }

   public void setHighLevelControllerCallbackForEstimator(Map<HighLevelControllerName, StateEstimatorMode> estimatorModeMap)
   {
      estimatorModeMapReference.set(estimatorModeMap);
   }

   public void addSmoothTransitionState(String transitionName,
                                        HighLevelControllerName transitionStateEnum,
                                        HighLevelControllerName currentControlStateEnum,
                                        HighLevelControllerName nextControlStateEnum,
                                        CommandBlenderFactory commandBlenderFactory)
   {
      controllerFactory.addCustomSmoothTransitionControlState(transitionName,
                                                              transitionStateEnum,
                                                              currentControlStateEnum,
                                                              nextControlStateEnum,
                                                              commandBlenderFactory);
   }

   public HighLevelHumanoidControllerFactory getControllerFactory()
   {
      return controllerFactory;
   }
}
