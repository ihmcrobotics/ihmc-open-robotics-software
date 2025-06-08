package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.JoystickBasedSteppingPluginFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import java.util.*;
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
   private static final boolean DISABLE_STEP_GENERATOR_THREAD = false;

   private final YoRegistry rootRegistry;

   // Robot model
   private final DRCRobotModel robotModel;

   // Hardware communication API
   private final HardwareCommunicationInterface hardwareCommunicationInterface;

   // ROS stuff
   public static final PriorityParameters ros2Priority = new PriorityParameters(25);
   public final String IHMC_ROS_STATE_ESTIMATOR_NODE_NAME;
   public final String IHMC_ROS_CONTROLLER_NODE_NAME;
   private final RealtimeROS2Node estimatorRealtimeROS2Node;
   private final RealtimeROS2Node controllerRealtimeROS2Node;

   // The thread factories
   private final AvatarEstimatorThreadFactory estimatorThreadFactory;
   private final HighLevelHumanoidControllerFactory controllerFactory;

   // The threads
   private final RequiredFactoryField<AvatarEstimatorThread> estimatorThread = new RequiredFactoryField<>("AvatarEstimatorThread");
   private final RequiredFactoryField<AvatarControllerThread> controllerThread = new RequiredFactoryField<>("AvatarControllerThread");
   private final RequiredFactoryField<AvatarStepGeneratorThread> stepGeneratorThread = new RequiredFactoryField<>("AvatarStepGeneratorThread");

   // Multi-threading manager
   private final RequiredFactoryField<AvatarMultiThreadingManager> threadingManager = new RequiredFactoryField<>("AvatarMultiThreadingManager");

   // Output processor
   private final AvatarLowLevelOutputProcessor lowLevelOutputProcessor;

   // The remaining constructor arguments
   private final MonotonicTime period;
   private final double schedulerDt;
   private final TimestampProvider monotonicTimeProvider;
   private final AvatarAffinityInterface affinity;
   private final boolean useRealtimeThreads;
   private final boolean useMultiThreading;
   private final YoVariableServer yoVariableServer;

   public AvatarMultiThreadingFactory(DRCRobotModel robotModel,
                                      FullHumanoidRobotModel fullRobotModel,
                                      HardwareCommunicationInterface hardwareCommunicationInterface,
                                      SensorReaderFactory sensorReaderFactory,
                                      HighLevelControllerStateFactory standPrepStateFactory,
                                      HighLevelControllerStateFactory freezeStateFactory,
                                      AvatarAffinityInterface affinity,
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      MonotonicTime period,
                                      double schedulerDt,
                                      TimestampProvider monotonicTimeProvider,
                                      YoRegistry registry,
                                      YoVariableServer yoVariableServer)
   {
      this.robotModel = robotModel;
      this.period = period;
      this.schedulerDt = schedulerDt;
      this.monotonicTimeProvider = monotonicTimeProvider;
      this.hardwareCommunicationInterface = hardwareCommunicationInterface;
      this.affinity = affinity;
      this.useRealtimeThreads = useRealtimeThreads;
      this.useMultiThreading = useMultiThreading;
      this.rootRegistry = registry;
      this.yoVariableServer = yoVariableServer;

      // Estimator and controller ROS2 nodes
      // PeriodicRealtimeThreadSchedulerFactory ros2RealtimeThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(ros2Priority);
      IHMC_ROS_STATE_ESTIMATOR_NODE_NAME = robotModel.getSimpleRobotName().toLowerCase() + "_ihmc_state_estimator";
      IHMC_ROS_CONTROLLER_NODE_NAME =robotModel.getSimpleRobotName().toLowerCase() + "_" + HumanoidControllerAPI.HUMANOID_CONTROL_MODULE_NAME;
      PeriodicNonRealtimeThreadSchedulerFactory ros2RealtimeThreadFactory = new PeriodicNonRealtimeThreadSchedulerFactory();
      estimatorRealtimeROS2Node = new ROS2NodeBuilder().buildRealtime(IHMC_ROS_STATE_ESTIMATOR_NODE_NAME, ros2RealtimeThreadFactory);
      controllerRealtimeROS2Node = new ROS2NodeBuilder().buildRealtime(IHMC_ROS_CONTROLLER_NODE_NAME, ros2RealtimeThreadFactory);

      // Set up low-level output processor
      lowLevelOutputProcessor = new AvatarLowLevelOutputProcessor(robotModel.getSimpleRobotName().toLowerCase(), fullRobotModel.getControllableOneDoFJoints(), schedulerDt, registry);

      // Setup state estimator factory
      estimatorThreadFactory = createStateEstimatorFactory(robotModel, fullRobotModel, sensorReaderFactory);

      // Setup state controller factory
      controllerFactory = createHighLevelControllerFactory(robotModel,
                                                           controllerRealtimeROS2Node,
                                                           lowLevelOutputProcessor,
                                                           standPrepStateFactory,
                                                           freezeStateFactory);

      // Add shutdown hook to kill the ROS nodes and various threads
      Runtime.getRuntime().addShutdownHook(new Thread(AvatarMultiThreadingFactory.this::destroy));
   }

   public void start()
   {
      estimatorRealtimeROS2Node.spin();
      controllerRealtimeROS2Node.spin();
      hardwareCommunicationInterface.start();
      threadingManager.get().start();
   }

   public void join()
   {
      threadingManager.get().join();
   }

   public void stop()
   {
      LogTools.info("Calling shutdown in the controller factory");
      estimatorRealtimeROS2Node.stopSpinning();
      controllerRealtimeROS2Node.stopSpinning();
      hardwareCommunicationInterface.stop();
      threadingManager.get().stop();
   }

   public void destroy()
   {
      this.stop();
      estimatorRealtimeROS2Node.destroy();
      controllerRealtimeROS2Node.destroy();
      hardwareCommunicationInterface.destroy();
      threadingManager.get().destroy();
   }

   public void buildThreads()
   {
      // Create estimator thread
      estimatorThread.set(estimatorThreadFactory.createAvatarEstimatorThread());


      // Create controller thread
      HashMap<HighLevelControllerName, StateEstimatorMode> stateModeMap = new HashMap<>();
      Arrays.stream(HighLevelControllerName.values).forEach(name -> stateModeMap.put(name, StateEstimatorMode.FROZEN));
      stateModeMap.put(STAND_TRANSITION_STATE, StateEstimatorMode.NORMAL);
      stateModeMap.put(EXIT_WALKING, StateEstimatorMode.NORMAL);
      stateModeMap.put(WALKING, StateEstimatorMode.NORMAL);
      estimatorThread.get().setupHighLevelControllerCallback(controllerFactory, stateModeMap);

      HumanoidRobotContextDataFactory controllerContextFactory = new HumanoidRobotContextDataFactory();
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
      stepGeneratorThread.set(createStepGeneratorThread(robotModel, controllerThread.get(), controllerContextFactory, controllerFactory));

      // Do some YoVariable stuff
      rootRegistry.addChild(estimatorThread.get().getYoRegistry());
      yoVariableServer.setMainRegistry(rootRegistry,
                                       estimatorThread.get().getFullRobotModel().getRootJoint().subtreeList(),
                                       estimatorThread.get().getSCS1YoGraphicsListRegistry());
//      yoVariableServer.addRegistry(estimatorThread.get().getYoRegistry(), estimatorThread.get().getSCS1YoGraphicsListRegistry());
      yoVariableServer.addRegistry(controllerThread.get().getYoVariableRegistry(), controllerThread.get().getSCS1YoGraphicsListRegistry());
      if (!DISABLE_STEP_GENERATOR_THREAD)
         yoVariableServer.addRegistry(stepGeneratorThread.get().getYoVariableRegistry(), stepGeneratorThread.get().getSCS1YoGraphicsListRegistry());

      // Create threading manager
      threadingManager.set(new AvatarMultiThreadingManager(robotModel.getSimpleRobotName().toLowerCase(),
                                                           robotModel,
                                                           estimatorThread.get().getHumanoidRobotContextData(),
                                                           estimatorThread.get().getFullRobotModel(),
                                                           hardwareCommunicationInterface,
                                                           lowLevelOutputProcessor,
                                                           estimatorThread.get(),
                                                           controllerThread.get(),
                                                           stepGeneratorThread.get(),
                                                           affinity,
                                                           schedulerDt,
                                                           period,
                                                           monotonicTimeProvider,
                                                           useRealtimeThreads,
                                                           useMultiThreading,
                                                           yoVariableServer,
                                                           rootRegistry));
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
//      avatarEstimatorThreadFactory.setYoGraphicsListRegistry(sensorReaderFactory.getYoGraphicsListRegistry()); //TODO do we need this?
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

         controllerFactory.addFinishedTransition(STAND_TRANSITION_STATE, WALKING, false);
         controllerFactory.addFinishedTransition(EXIT_WALKING, FREEZE_STATE);

         controllerFactory.addCustomStateTransition(createStandTransitionState(controllerFactory, feetForceSensorNames));

         // Transition to Freeze state if we are unservoing
         HighLevelControllerStateCommand transitionToFreezeCommand = new HighLevelControllerStateCommand();
         CommandInputManager controllerCommandInputManager = controllerFactory.getCommandInputManager();
         lowLevelOutputProcessor.addServoListener(value ->
                                                  {
                                                     transitionToFreezeCommand.setHighLevelControllerName(FREEZE_STATE);
                                                     if (value.getValueAsDouble() == 0.0)
                                                        controllerCommandInputManager.submitMessage(transitionToFreezeCommand);
                                                  });

         // Transition to Freeze state if we are toggling E-Stop on or off
         hardwareCommunicationInterface.addSoftEStopListener(value ->
                                                             {
                                                                transitionToFreezeCommand.setHighLevelControllerName(FREEZE_STATE);
                                                                controllerCommandInputManager.submitMessage(transitionToFreezeCommand);
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
      YoGraphicsListRegistry stepGeneratorGraphics = null;

      LogTools.info("create step generator = " + !DISABLE_STEP_GENERATOR_THREAD);

      HumanoidSteppingPluginEnvironmentalConstraints environmentalConstraints = new HumanoidSteppingPluginEnvironmentalConstraints(robotModel.getContactPointParameters(),
                                                                                                                                   robotModel.getWalkingControllerParameters()
                                                                                                                                             .getSteppingParameters(),
                                                                                                                                   robotModel.getSteppingEnvironmentalConstraintParameters());

      controllerFactory.setListenToHighLevelStatePackets(true);

      JoystickBasedSteppingPluginFactory pluginFactory = new JoystickBasedSteppingPluginFactory();

      if (DISABLE_STEP_GENERATOR_THREAD)
      {
         // sets up the environmental constraint manager as a planar region consumer in the input manager
         pluginFactory.addPlanarRegionsListCommandConsumer(environmentalConstraints);
         // Adds functions that adjust the footholds based on the environment.
         pluginFactory.setFootStepPlanAdjustment(environmentalConstraints.getFootstepPlanAdjustment());
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
         List<ArtifactList> artifactLists = new ArrayList<>();
         environmentalConstraints.getGraphicsListRegistry().getRegisteredArtifactLists(artifactLists);
         controllerThread.getSCS1YoGraphicsListRegistry().registerYoGraphicsLists(environmentalConstraints.getGraphicsListRegistry().getYoGraphicsLists());
         controllerThread.getSCS1YoGraphicsListRegistry().registerArtifactLists(artifactLists);
      }
      else
      {
         stepGeneratorGraphics = new YoGraphicsListRegistry();

         stepGeneratorThread = new AvatarStepGeneratorThread(pluginFactory,
                                                             controllerContextFactory,
                                                             controllerFactory.getStatusOutputManager(),
                                                             controllerFactory.getCommandInputManager(),
                                                             robotModel,
                                                             environmentalConstraints,
                                                             controllerRealtimeROS2Node);
      }

      return stepGeneratorThread;
   }

   /**
    * The transition to the STAND_TRANSITION state will happen when: 1- the STAND_PREP is done, i.e.
    * the ramp up ratio is at 1, AND 2- the feet loaded transition is satisfied is requested via the
    * YoEnum.
    */
   private static ControllerStateTransitionFactory<HighLevelControllerName> createStandTransitionState(HighLevelHumanoidControllerFactory controllerFactory,
                                                                                                       SideDependentList<String> feetForceSensorNames)
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
            double controlDT = controllerToolbox.getControlDT();
            YoEnum<HighLevelControllerName> requestedState = controllerFactory.getRequestedControlStateEnum();
            ForceSensorDataHolderReadOnly forceSensorDataHolder = controllerFactoryHelper.getForceSensorDataHolder();
            HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();

            StateTransitionCondition feetLoadedTransition = new FeetLoadedToWalkingStandTransition(STAND_TRANSITION_STATE,
                                                                                                   requestedState,
                                                                                                   forceSensorDataHolder,
                                                                                                   feetForceSensorNames,
                                                                                                   controlDT,
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

            return new StateTransition<HighLevelControllerName>(STAND_TRANSITION_STATE, condition);
         }
      };
   }

   public void addCustomControlState(HighLevelControllerStateFactory customControllerStateFactory)
   {
      controllerFactory.addCustomControlState(customControllerStateFactory);
   }

   public static List<JointBasics> createYoVariableServerJointList(RigidBodyBasics rootBody)
   {
      List<JointBasics> joints = new ArrayList<>();

      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         if (joint instanceof CrossFourBarJoint)
         {
            joints.addAll(((CrossFourBarJoint) joint).getFourBarFunction().getLoopJoints());
         }
         else
         {
            joints.add(joint);
         }
      }

      return joints;
   }

   public YoRegistry getEstimatorRegistry()
   {
      return estimatorThread.get().getYoRegistry();
   }

   public YoGraphicGroupDefinition getEstimatorYoGraphics()
   {
      return estimatorThread.get().getSCS2YoGraphics();
   }

   public YoRegistry getControllerRegistry()
   {
      return controllerThread.get().getYoVariableRegistry();
   }

   public YoGraphicGroupDefinition getControllerYoGraphics()
   {
      return controllerThread.get().getSCS2YoGraphics();
   }

   public YoRegistry getStepGeneratorRegistry()
   {
      return stepGeneratorThread.get().getYoVariableRegistry();
   }

   public YoGraphicGroupDefinition getStepGeneratorYoGraphics()
   {
      return stepGeneratorThread.get().getSCS2YoGraphics();
   }

   public AvatarMultiThreadingManager getThreadingManager()
   {
      return threadingManager.get();
   }
}
