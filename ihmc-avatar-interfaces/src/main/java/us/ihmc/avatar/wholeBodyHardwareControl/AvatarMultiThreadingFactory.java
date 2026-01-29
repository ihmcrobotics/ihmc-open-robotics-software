package us.ihmc.avatar.wholeBodyHardwareControl;

import org.apache.commons.math3.util.Precision;
import us.ihmc.affinity.Processor;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.IKStreamingRTPluginFactory;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommandBlenderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPluginFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.JoystickBasedSteppingPlugin;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.JoystickBasedSteppingPluginFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
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

import static us.ihmc.avatar.wholeBodyHardwareControl.AvatarMultiThreadingManager.runAll;
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
   private final RealtimeROS2Node estimatorRealtimeROS2Node;
   private final RealtimeROS2Node controllerRealtimeROS2Node;
   private final PeriodicThreadSchedulerFactory ros2ThreadFactory;

   // The thread factories
   private final HighLevelHumanoidControllerFactory controllerFactory;

   // The primary thread runnables for each thread
   private final AvatarEstimatorThread avatarEstimatorThread;
   private final OptionalFactoryField<AvatarStepGeneratorThread> avatarStepGeneratorThread = new OptionalFactoryField<>("AvatarStepGeneratorThread");
   private final OptionalFactoryField<Map<HighLevelControllerName, StateEstimatorMode>> estimatorModeMapReference = new OptionalFactoryField<>("estimatorModeMapReference");
   private final OptionalFactoryField<IKStreamingRTPluginFactory.IKStreamingRTThread> avatarIKStreamingThread = new OptionalFactoryField<>("AvatarIKStreamingThread");

   // Some stuff for the step generator
   private final HumanoidSteppingPluginEnvironmentalConstraints environmentalConstraints;
   private final HumanoidSteppingPluginFactory humanoidSteppingPluginFactory;

   // The pre and post runnables for controller and estimator threads
   private final List<Runnable> preControllerRunnables = new ArrayList<>();
   private final List<Runnable> postControllerRunnables = new ArrayList<>();
   private final List<Runnable> preEstimatorRunnables = new ArrayList<>();
   private final List<Runnable> postEstimatorRunnables = new ArrayList<>();

   // The child threads and tasks
   private final List<Runnable> threads = new ArrayList<>();
   private final List<HumanoidRobotControlTask> tasks = new ArrayList<>();

   // Optional externally set master thread
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
                                      boolean useRealtimeThreads,
                                      boolean useMultiThreading,
                                      boolean useLocalLogging,
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
      this.useLocalLogging = useLocalLogging;
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

      estimatorRealtimeROS2Node = new ROS2NodeBuilder().buildRealtime(IHMC_ROS_STATE_ESTIMATOR_NODE_NAME, ros2ThreadFactory);
      controllerRealtimeROS2Node = new ROS2NodeBuilder().buildRealtime(IHMC_ROS_CONTROLLER_NODE_NAME, ros2ThreadFactory);

      // Create estimator thread
      avatarEstimatorThread = createAvatarEstimatorThread(robotModel, sensorReaderFactory);
      setupEstimatorTaskAndThread(masterRobotModel, avatarEstimatorThread, masterFullRobotModel, yoVariableServer);

      // Setup state controller factory
      controllerFactory = createHighLevelControllerFactory(robotModel,
                                                           controllerRealtimeROS2Node,
                                                           lowLevelOutputProcessor,
                                                           standPrepStateFactory,
                                                           freezeStateFactory);
      controllerFactory.setListenToHighLevelStatePackets(true);

      // Some extra stuff for the step generator
      humanoidSteppingPluginFactory = new JoystickBasedSteppingPluginFactory();
      environmentalConstraints = new HumanoidSteppingPluginEnvironmentalConstraints(masterRobotModel.getContactPointParameters(),
                                                                                    masterRobotModel.getWalkingControllerParameters().getSteppingParametersForStepGeneration());

      // Set the root registry as the YoVariableServer's main registry
      yoVariableServer.setMainRegistry(rootRegistry,
                                       masterFullRobotModel.getRootJoint().subtreeList(),
                                       null,
                                       new YoGraphicGroupDefinition(rootRegistry.getName()));
   }

   public AvatarMultiThreadingManager buildThreadsAndThreadingManager()
   {
      // Create controller thread
      if (estimatorModeMapReference.hasValue())
      {
         avatarEstimatorThread.setupHighLevelControllerCallback(controllerFactory, estimatorModeMapReference.get());
      }
      else
      {
         HashMap<HighLevelControllerName, StateEstimatorMode> stateModeMap = new HashMap<>();
         Arrays.stream(HighLevelControllerName.values).forEach(name -> stateModeMap.put(name, StateEstimatorMode.FROZEN));
         stateModeMap.put(STAND_TRANSITION_STATE, StateEstimatorMode.NORMAL);
         stateModeMap.put(EXIT_WALKING, StateEstimatorMode.NORMAL);
         stateModeMap.put(WALKING, StateEstimatorMode.NORMAL);
         avatarEstimatorThread.setupHighLevelControllerCallback(controllerFactory, stateModeMap);
      }
      AvatarControllerThread avatarControllerThread = new AvatarControllerThread(masterRobotModel.getSimpleRobotName().toLowerCase(), masterRobotModel,
                                                                                 null,
                                                                                 masterRobotModel.getSensorInformation(),
                                                                                 controllerFactory,
                                                                                 controllerContextFactory,
                                                                                 null,
                                                                                 controllerRealtimeROS2Node,
                                                                                 GRAVITY,
                                                                                 false);
      setupControllerTaskAndThread(masterRobotModel, avatarControllerThread, masterFullRobotModel, yoVariableServer);

      // Add estimator thread registry directly to the YoVariableServer (since it is in a separate thread)
      yoVariableServer.addRegistry(avatarEstimatorThread.getYoRegistry(), null, avatarEstimatorThread.getSCS2YoGraphics());

      // Add controller thread registry directly to the YoVariableServer (since it is in a separate thread)
      yoVariableServer.addRegistry(avatarControllerThread.getYoVariableRegistry(), null, avatarControllerThread.getSCS2YoGraphics());

      // Add step generator thread registry directly to the YoVariableServer (since it is in a separate thread)
      if (avatarStepGeneratorThread.hasValue())
      {
         yoVariableServer.addRegistry(avatarStepGeneratorThread.get().getYoVariableRegistry(), null, avatarStepGeneratorThread.get().getSCS2YoGraphics());
      }
      else
      {
         // sets up the environmental constraint manager as a height map consumer in the input manager
         humanoidSteppingPluginFactory.addHeightMapCommandConsumer(environmentalConstraints);
         // Adds functions that adjust the footholds based on the environment.
         humanoidSteppingPluginFactory.setFootStepAdjustment(environmentalConstraints.getFootstepAdjustment());
         // Adds checkers for footholds based on the environment
         for (FootstepValidityIndicator footstepValidityIndicator : environmentalConstraints.getFootstepValidityIndicators())
            humanoidSteppingPluginFactory.addFootstepValidityIndicator(footstepValidityIndicator);
         // clear the environment at the beginning of every update
         humanoidSteppingPluginFactory.addUpdatable(environmentalConstraints);

         // create the callback listeners for the planar regions in the stepping plugin
         humanoidSteppingPluginFactory.createStepGeneratorNetworkSubscriber(masterRobotModel.getSimpleRobotName().toLowerCase(), controllerRealtimeROS2Node);

         controllerFactory.addControllerPlugin(humanoidSteppingPluginFactory);

         // otherwise this would go into the step generator registry
         avatarControllerThread.getYoVariableRegistry().addChild(environmentalConstraints.getRegistry());
      }

      // Add IK streaming thread registry directly to the YoVariableServer (since it is in a separate thread)
      if (avatarIKStreamingThread.hasValue())
         yoVariableServer.addRegistry(avatarIKStreamingThread.get().getYoVariableRegistry(), null, avatarIKStreamingThread.get().getSCS2YoGraphics());

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

      //
      threadingManager.get().addPostMasterThreadRunnable(()->HumanoidRobotContextTools.updateRobot(masterFullRobotModel, masterContext.getProcessedJointData()));

      if (useLocalLogging)
      {
         // Setup logger
         ArrayList<RegistrySendBufferBuilder> builders = new ArrayList<>();
         builders.add(new RegistrySendBufferBuilder(rootRegistry,
                                                    masterFullRobotModel.getRootJoint().subtreeList(),
                                                    null,
                                                    avatarEstimatorThread.getSCS2YoGraphics()));
         builders.add(new RegistrySendBufferBuilder(avatarControllerThread.getYoVariableRegistry(),
                                                    null,
                                                    avatarControllerThread.getSCS2YoGraphics()));
         if (avatarStepGeneratorThread.hasValue())
         {
            builders.add(new RegistrySendBufferBuilder(avatarStepGeneratorThread.get().getYoVariableRegistry(),
                                                       null,
                                                       avatarStepGeneratorThread.get().getSCS2YoGraphics()));
         }
         if (avatarIKStreamingThread.hasValue())
         {
            builders.add(new RegistrySendBufferBuilder(avatarIKStreamingThread.get().getYoVariableRegistry(),
                                                       null,
                                                       avatarIKStreamingThread.get().getSCS2YoGraphics()));
         }

         // FIXME add this back when a release of the logger is done.
//         builders.add(new RegistrySendBufferBuilder(jvmStatisticsGenerator.getYoRegistry(), null));

         // Logging locally on the robot
         IntraprocessYoVariableLogger intraprocessYoVariableLogger = new IntraprocessYoVariableLogger(builders,
                                                                                                      masterRobotModel.getEstimatorDT(),
                                                                                                      masterRobotModel.getSimpleRobotName().toLowerCase() + getClass().getSimpleName(), masterRobotModel.getLogModelProvider());

         if (intraprocessYoVariableLogger.create())
         {
            LogTools.info("[Logging] Logging locally to disk");

            threadingManager.get()
                            .addPostMasterThreadRunnable(() -> intraprocessYoVariableLogger.update(avatarEstimatorThread.getHumanoidRobotContextData()
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
   private AvatarEstimatorThread createAvatarEstimatorThread(DRCRobotModel robotModel, SensorReaderFactory sensorReaderFactory)
   {
      LogTools.info("The Squirrel estimates the number of acorns he needs for winter. Not many in Florida he thinks");
      HumanoidRobotContextDataFactory estimatorContextDataFactory = new HumanoidRobotContextDataFactory();

      AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
      avatarEstimatorThreadFactory.setROS2Info(estimatorRealtimeROS2Node, robotModel.getSimpleRobotName());

      avatarEstimatorThreadFactory.configureWithWholeBodyControllerParameters(robotModel);
      avatarEstimatorThreadFactory.configureWithDRCRobotModel(robotModel);
      avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
      avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(estimatorContextDataFactory);
      avatarEstimatorThreadFactory.setGravity(GRAVITY);
      //      if (secondaryEstimatorFactory != null)
      //         avatarEstimatorThreadFactory.addSecondaryStateEstimatorFactory(secondaryEstimatorFactory);
      StateEstimatorController stateEstimator = avatarEstimatorThreadFactory.getMainStateEstimator();

      return avatarEstimatorThreadFactory.createAvatarEstimatorThread();
   }

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
      estimatorTask.addRunnableOnStartup(() ->
                                          {
                                             estimatorRealtimeROS2Node.spin();
                                             LogTools.info("Estimator node has started spinning");
                                          });

      // Add estimator cleanup callback to stop spinning node
      estimatorTask.addRunnableOnCleanup(() ->
                                          {
                                             estimatorRealtimeROS2Node.stopSpinning();
                                             LogTools.info("Estimator node has stopped Spinning");
                                          });

      // Add estimator cleanup callback to destroy node
      estimatorTask.addRunnableOnCleanup(() ->
                                          {
                                             estimatorRealtimeROS2Node.destroy();
                                             LogTools.info("Estimator node has been destroyed");
                                          });

      // Set up estimator realtime or non-realtime thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread estimatorRealtimeThread = new RealtimeThread(affinity.getEstimatorThreadPriority(),
                                                                      estimatorTask,
                                                                      estimatorTask.getClass().getSimpleName() + "Thread");
         estimatorRealtimeThread.setAffinity(affinity.getEstimatorThreadProcessor());
         estimatorRealtimeThread.start();

         threads.add(estimatorRealtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread estimatorNonRealtimeThread = new Thread(estimatorTask, estimatorTask.getClass().getSimpleName() + "Thread");
         estimatorNonRealtimeThread.start();

         threads.add(estimatorNonRealtimeThread);
      }

      tasks.add(estimatorTask);
      return estimatorTask;
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

         controllerFactory.addCustomStateTransition(createStandTransitionState(STAND_TRANSITION_STATE, controllerFactory,  !highLevelControllerParameters.automaticallyTransitionToWalkingWhenReady()));

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

   private HumanoidRobotControlTask setupControllerTaskAndThread(DRCRobotModel robotModel,
                                                                 AvatarControllerThread controllerThread,
                                                                 FullHumanoidRobotModel masterFullRobotModel,
                                                                 YoVariableServer yoVariableServer)
   {
      // Set up Controller Task
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / masterThreadDt);
      if (!Precision.equals(robotModel.getControllerDT() / masterThreadDt, controllerDivisor))
         throw new RuntimeException("Controller DT must be multiple of master thread DT.");

      ControllerTask controllerTask = new ControllerTask("Controller", controllerThread, controllerDivisor, masterThreadDt, masterFullRobotModel);

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
      controllerTask.addRunnableOnStartup(() ->
                                       {
                                          controllerRealtimeROS2Node.spin();
                                          LogTools.info("Controller node has started spinning");
                                       });

      // Add controller cleanup callback to stop spinning node
      controllerTask.addRunnableOnCleanup(() ->
                                       {
                                          controllerRealtimeROS2Node.stopSpinning();
                                          LogTools.info("Controller node has stopped Spinning");
                                       });

      // Add controller cleanup callback to destroy node
      controllerTask.addRunnableOnCleanup(() ->
                                       {
                                          controllerRealtimeROS2Node.destroy();
                                          LogTools.info("Controller node has been destroyed");
                                       });

      // Set up controller realtime or non-realtime thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread controllerRealtimeThread = new RealtimeThread(affinity.getControllerThreadPriority(),
                                                                      controllerTask,
                                                                      controllerTask.getClass().getSimpleName() + "Thread");
         controllerRealtimeThread.setAffinity(affinity.getControllerThreadProcessor());
         controllerRealtimeThread.start();

         threads.add(controllerRealtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread controllerNonRealtimeThread = new Thread(controllerTask, controllerTask.getClass().getSimpleName() + "Thread");
         controllerNonRealtimeThread.start();

         threads.add(controllerNonRealtimeThread);
      }

      tasks.add(controllerTask);
      return controllerTask;
   }

   /**
    * Create Step Generator Thread
    */
   public void addStepGeneratorThread()
   {
      AvatarStepGeneratorThread stepGeneratorThread = new AvatarStepGeneratorThread(humanoidSteppingPluginFactory,
                                                                                    controllerContextFactory,
                                                                                    controllerFactory.getStatusOutputManager(),
                                                                                    controllerFactory.getCommandInputManager(),
                                                                                    masterRobotModel,
                                                                                    environmentalConstraints,
                                                                                    controllerRealtimeROS2Node);

      avatarStepGeneratorThread.set(stepGeneratorThread);
      setupStepGeneratorTaskAndThread(masterRobotModel,
                                      stepGeneratorThread,
                                      masterFullRobotModel,
                                      yoVariableServer);
   }

   private HumanoidRobotControlTask setupStepGeneratorTaskAndThread(DRCRobotModel robotModel,
                                                                    AvatarStepGeneratorThread stepGeneratorThread,
                                                                    FullHumanoidRobotModel masterFullRobotModel,
                                                                    YoVariableServer yoVariableServer)
   {
      // Set up Step Generator Task
      int stepGeneratorDivisor = (int) Math.round(robotModel.getStepGeneratorDT() / masterThreadDt);
      if (!Precision.equals(robotModel.getStepGeneratorDT() / masterThreadDt, stepGeneratorDivisor))
         throw new RuntimeException("Step generator DT must be multiple of master thread DT.");

      StepGeneratorTask stepGeneratorTask = new StepGeneratorTask("StepGenerator", stepGeneratorThread, stepGeneratorDivisor, masterThreadDt, masterFullRobotModel);

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
      stepGeneratorTask.addRunnableOnCleanup(avatarStepGeneratorThread.get()::destroy);

      // Set up step generator realtime or non-realtime thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread stepGeneratorRealtimeThread = new RealtimeThread(affinity.getStepGeneratorThreadPriority(),
                                                                         stepGeneratorTask,
                                                                         stepGeneratorTask.getClass().getSimpleName() + "Thread");
         stepGeneratorRealtimeThread.setAffinity(affinity.getStepGeneratorThreadProcessor());
         stepGeneratorRealtimeThread.start();

         threads.add(stepGeneratorRealtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread stepGeneratorNonRealtimeThread = new Thread(stepGeneratorTask, stepGeneratorTask.getClass().getSimpleName() + "Thread");
         stepGeneratorNonRealtimeThread.start();

         threads.add(stepGeneratorNonRealtimeThread);
      }

      tasks.add(stepGeneratorTask);
      return stepGeneratorTask;
   }

   public void addIKStreamingThread(KinematicsStreamingToolboxParameters ikStreamingParameters)
   {
      avatarIKStreamingThread.set(new IKStreamingRTPluginFactory().createRTThread(masterRobotModel.getSimpleRobotName(),
                                                                                  estimatorRealtimeROS2Node,
                                                                                  controllerFactory.getCommandInputManager(),
                                                                                  controllerFactory.getStatusOutputManager(), masterRobotModel,
                                                                                  controllerContextFactory,
                                                                                  masterRobotModel.getHumanoidRobotKinematicsCollisionModel(),
                                                                                  ikStreamingParameters));

      setupIKStreamingTaskAndThread(avatarIKStreamingThread.get(), yoVariableServer);
   }

   private HumanoidRobotControlTask setupIKStreamingTaskAndThread(IKStreamingRTPluginFactory.IKStreamingRTThread ikStreamingThread,
                                                                  YoVariableServer yoVariableServer)
   {
      // Set up Step Generator Task
      IKStreamingRTPluginFactory.IKStreamingRTTask ikStreamingTask = IKStreamingRTPluginFactory.createIKStreamingRTTask(ikStreamingThread, masterThreadDt);

      // Add post-IK streaming callback to update YoVariable server with IK streaming registry
      if (yoVariableServer != null)
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

      // Set up IK streaming realtime or non-realtime thread
      if (useRealtimeThreads && useMultiThreading)
      {
         RealtimeThread ikStreamingRealtimeThread = new RealtimeThread(affinity.getIKStreamingThreadPriority(),
                                                                       ikStreamingTask,
                                                                       ikStreamingTask.getClass().getSimpleName() + "Thread");
         ikStreamingRealtimeThread.setAffinity(affinity.getIKStreamingThreadProcessor());
         ikStreamingRealtimeThread.start();

         threads.add(ikStreamingRealtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread ikStreamingNonRealtimeThread = new Thread(ikStreamingTask, ikStreamingTask.getClass().getSimpleName() + "Thread");
         ikStreamingNonRealtimeThread.start();

         threads.add(ikStreamingNonRealtimeThread);
      }

      tasks.add(ikStreamingTask);
      return ikStreamingTask;
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
            double controlDT = controllerToolbox.getControlDT();
            YoEnum<HighLevelControllerName> requestedState = controllerFactory.getRequestedControlStateEnum();
            HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();

            StateTransitionCondition feetLoadedTransition = new FeetLoadedToWalkingStandTransition(transitionStateName,
                                                                                                   requestedState,
                                                                                                   waitForRequestToTransition,
                                                                                                   controllerToolbox.getFootSwitches(),
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

   public Runnable addThread(String name, Runnable runnable, double threadDt, int priority, int core, YoRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry)
   {
      // Set up Task
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
      yoVariableServer.addRegistry(registry, yoGraphicsRegistry);

      // Add post-thread callback to update YoVariable server with thread registry
      if (yoVariableServer != null)
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

      Runnable retThread = null;
      
      // Set up realtime or non-realtime thread
      if (useRealtimeThreads && useMultiThreading)
      {
         Processor threadProcessor = affinity.getSocket().getCore(affinity.checkCoreIsValid(core)).getDefaultProcessor();
         PriorityParameters threadPriority = new PriorityParameters(priority);

         RealtimeThread realtimeThread = new RealtimeThread(threadPriority, task, name + "Thread");
         realtimeThread.setAffinity(threadProcessor);
         realtimeThread.start();
         
         retThread = realtimeThread;
         addThread(realtimeThread);
      }
      else if (!useRealtimeThreads && useMultiThreading)
      {
         Thread nonRealtimeThread = new Thread(task, name + "Thread");
         nonRealtimeThread.start();

         retThread = nonRealtimeThread;
         addThread(nonRealtimeThread);
      }

      addTask(task);
      return retThread;
   }

   public void addTask(HumanoidRobotControlTask task)
   {
      tasks.add(task);
   }

   public void addThread(Runnable thread)
   {
      threads.add(thread);
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
      return controllerFactory;
   }

   public YoRegistry getEstimatorRegistry()
   {
      return avatarEstimatorThread.getYoRegistry();
   }

   public YoGraphicGroupDefinition getEstimatorYoGraphics()
   {
      return avatarEstimatorThread.getSCS2YoGraphics();
   }

   public RealtimeROS2Node getEstimatorROS2Node()
   {
      return estimatorRealtimeROS2Node;
   }

   public FullHumanoidRobotModel getEstimatorFullRobotModel()
   {
      return avatarEstimatorThread.getFullRobotModel();
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
