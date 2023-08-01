package us.ihmc.valkyrieRosControl.upperBody;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HoldPositionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.*;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrieRosControl.*;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.*;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.*;
import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME;

public class ValkyrieUpperBodyController extends IHMCWholeRobotControlJavaBridge
{
   private static final boolean ARM_MASS_SIMULATORS = true;

   private static final String[] torqueControlledJoints;
   private static final String[] positionControlledJoints = {"lowerNeckPitch", "neckYaw", "upperNeckPitch"};
   private static final String[] allValkyrieJoints;

   static
   {
      List<String> torqueControlledJointList = new ArrayList<>();

      // Torso joints
      torqueControlledJointList.add("torsoYaw");
      torqueControlledJointList.add("torsoPitch");
      torqueControlledJointList.add("torsoRoll");

      for (RobotSide robotSide : RobotSide.values)
      {
         // Upper arm joints
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ShoulderPitch");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ShoulderRoll");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ShoulderYaw");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ElbowPitch");

         if (ARM_MASS_SIMULATORS)
            continue;

         // Forearm joints
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ForearmYaw");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "WristRoll");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "WristPitch");

         // Finger joints
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "IndexFingerMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "MiddleFingerMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "PinkyMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ThumbMotorRoll");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ThumbMotorPitch1");
         torqueControlledJointList.add(robotSide.getLowerCaseName() + "ThumbMotorPitch2");
      }

      torqueControlledJoints = torqueControlledJointList.toArray(new String[0]);

      List<String> allJointsList = new ArrayList<>();
      Arrays.stream(torqueControlledJoints).forEach(allJointsList::add);
      Arrays.stream(positionControlledJoints).forEach(allJointsList::add);
      allValkyrieJoints = allJointsList.toArray(new String[0]);
   }

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private static final ValkyrieRobotVersion VERSION = ValkyrieRobotVersion.UPPER_BODY;
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, VERSION);
   private final FullHumanoidRobotModel fullRobotModel;
   private final ValkyrieUpperBodyStateEstimator stateEstimator;
   private ValkyrieUpperBodyOutputWriter outputWriter;
   private final RealtimeROS2Node ros2Node;
   private final RobotConfigurationDataPublisher robotConfigurationDataPublisher;

   private final YoDouble yoTime = new YoDouble("yoTime", registry);
   private final SettableTimestampProvider wallTimeProvider = new SettableTimestampProvider();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private final YoLowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;

   private final HashMap<String, EffortJointHandle> effortJointHandles = new HashMap<>();
   private final HashMap<String, PositionJointHandle> positionJointHandles = new HashMap<>();
   private final HashMap<String, JointStateHandle> jointStateHandles = new HashMap<>();

   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private YoVariableServer yoVariableServer;
   private final ValkyrieTorqueOffsetPrinter valkyrieTorqueOffsetPrinter = new ValkyrieTorqueOffsetPrinter();
   private StateMachine<HighLevelControllerName, HighLevelControllerState> controllerStateMachine;

   public ValkyrieUpperBodyController()
   {
      fullRobotModel = robotModel.createFullRobotModel();

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());
      JointBasics[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);

      CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      PeriodicRealtimeThreadSchedulerFactory realtimeThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY);
      ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, realtimeThreadFactory, VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME);
      stateEstimator = new ValkyrieUpperBodyStateEstimator(fullRobotModel.getRootJoint(), controlledOneDoFJoints, registry);

      jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);

      ROS2Topic<?> inputTopic = ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName());
      ROS2Topic<?> outputTopic = ROS2Tools.getControllerOutputTopic(robotModel.getSimpleRobotName());
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(inputTopic,
                                                                                                commandInputManager,
                                                                                                outputTopic,
                                                                                                statusMessageOutputManager,
                                                                                                ros2Node);
      controllerNetworkSubscriber.addMessageCollectors(ControllerAPIDefinition.createDefaultMessageIDExtractor(), 3);
      controllerNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());

      RobotConfigurationDataPublisherFactory robotConfigurationDataPublisherFactory = new RobotConfigurationDataPublisherFactory();
      robotConfigurationDataPublisherFactory.setDefinitionsToPublish(fullRobotModel);
      robotConfigurationDataPublisherFactory.setROS2Info(ros2Node, outputTopic);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(fullRobotModel.getForceSensorDefinitions());
      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      SensorProcessing sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, robotModel.getStateEstimatorParameters(), registry);
      robotConfigurationDataPublisherFactory.setSensorSource(fullRobotModel, forceSensorDataHolder, sensorProcessing);
      robotConfigurationDataPublisher = robotConfigurationDataPublisherFactory.createRobotConfigurationDataPublisher();

      ros2Node.spin();
   }

   @Override
   protected void init()
   {
      LogTools.info("Valkyrie robot version: " + VERSION);

      HashSet<String> torqueControlledJointsSet = new HashSet<>(Arrays.asList(torqueControlledJoints));
      HashSet<String> positionControlledJointsSet = new HashSet<>(Arrays.asList(positionControlledJoints));

      for (String jointName : allValkyrieJoints)
      {
         if (torqueControlledJointsSet.contains(jointName) && positionControlledJointsSet.contains(jointName))
         {
            throw new RuntimeException("Joint cannot be both position controlled and torque controlled via ROS Control! Joint name: " + jointName);
         }
         if (torqueControlledJointsSet.contains(jointName))
         {
            effortJointHandles.put(jointName, createEffortJointHandle(jointName));
         }
         else if (positionControlledJointsSet.contains(jointName))
         {
            positionJointHandles.put(jointName, createPositionJointHandle(jointName));
         }
         else
         {
            jointStateHandles.put(jointName, createJointStateHandle(jointName));
         }
      }

      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      String leftTrunkIMUSensor = ValkyrieSensorInformation.leftTrunkIMUSensor;
      imuHandles.put(leftTrunkIMUSensor, createIMUHandle(leftTrunkIMUSensor));

      // The upper body has no force-torque sensors
      HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles = new HashMap<>();

      ValkyrieSensorInformation sensorInformation = robotModel.getSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(wallTimeProvider,
                                                                                                            monotonicTimeProvider,
                                                                                                            stateEstimatorParameters,
                                                                                                            effortJointHandles,
                                                                                                            positionJointHandles,
                                                                                                            jointStateHandles,
                                                                                                            imuHandles,
                                                                                                            forceTorqueSensorHandles,
                                                                                                            robotModel.getJointMap(),
                                                                                                            sensorInformation);
      sensorReaderFactory.build(fullRobotModel.getRootJoint(), fullRobotModel.getIMUDefinitions(), fullRobotModel.getForceSensorDefinitions(), jointDesiredOutputList, registry);
      ValkyrieRosControlSensorReader sensorReader = sensorReaderFactory.getSensorReader();
      stateEstimator.init(sensorReader);

      // Setup upper body controller state machine
      StateMachineFactory<HighLevelControllerName, HighLevelControllerState> factory = new StateMachineFactory<>(HighLevelControllerName.class);
      factory.setNamePrefix("highLevelControllerName").setRegistry(registry).buildYoClock(yoTime);

      // Setup stand prep state
      StandPrepControllerState standPrepControllerState = new StandPrepControllerState(controlledOneDoFJoints,
                                                                                       robotModel.getHighLevelControllerParameters(),
                                                                                       jointDesiredOutputList,
                                                                                       yoTime);
      factory.addState(standPrepControllerState.getHighLevelControllerName(), standPrepControllerState);

      // Stand ready state
      HoldPositionControllerState standReadyState = new HoldPositionControllerState(HighLevelControllerName.STAND_READY, controlledOneDoFJoints,
                                                                                    robotModel.getHighLevelControllerParameters(),
                                                                                    jointDesiredOutputList);
      factory.addState(standReadyState.getHighLevelControllerName(), standReadyState);

      // Setup manipulation state
      ValkyrieUpperBodyManipulationState manipulationState = new ValkyrieUpperBodyManipulationState(robotModel.getHighLevelControllerParameters(),
                                                                                                    robotModel.getWalkingControllerParameters(),
                                                                                                    fullRobotModel,
                                                                                                    controlledOneDoFJoints,
                                                                                                    yoTime,
                                                                                                    graphicsListRegistry);

      // Smooth transition state
      SmoothTransitionControllerState standTransitionState = new SmoothTransitionControllerState("toWalking",
                                                                                                 HighLevelControllerName.STAND_TRANSITION_STATE,
                                                                                                 standReadyState,
                                                                                                 manipulationState, controlledOneDoFJoints,
                                                                                                 robotModel.getHighLevelControllerParameters());
      factory.addState(standTransitionState.getHighLevelControllerName(), standTransitionState);

      // Setup calibration state
      ValkyrieCalibrationControllerState calibrationControllerState = new ValkyrieCalibrationControllerState(null,
                                                                                                             fullRobotModel,
                                                                                                             controlledOneDoFJoints,
                                                                                                             yoTime,
                                                                                                             robotModel.getHighLevelControllerParameters(),
                                                                                                             jointDesiredOutputList,
                                                                                                             null,
                                                                                                             null,
                                                                                                             robotModel.getCalibrationParameters(),
                                                                                                             valkyrieTorqueOffsetPrinter);
      factory.addState(calibrationControllerState.getHighLevelControllerName(), calibrationControllerState);

      // Setup transitions
      YoEnum<HighLevelControllerName> requestedHighLevelControllerState = new YoEnum<>("requestedHighLevelControllerState", registry, HighLevelControllerName.class, true);

      // Stand prep -> Stand ready
      factory.addDoneTransition(standPrepControllerState.getHighLevelControllerName(), standReadyState.getHighLevelControllerName());

      // Calibration Request
      factory.addRequestedTransition(standPrepControllerState.getHighLevelControllerName(),     calibrationControllerState.getHighLevelControllerName(), requestedHighLevelControllerState);
      factory.addRequestedTransition(standReadyState.getHighLevelControllerName(),              calibrationControllerState.getHighLevelControllerName(), requestedHighLevelControllerState);

      // Calibration -> Stand prep
      factory.addDoneTransition(calibrationControllerState.getHighLevelControllerName(), standPrepControllerState.getHighLevelControllerName());

//      factory.addRequestedTransition(calibrationControllerState.getHighLevelControllerName(),   standPrepControllerState.getHighLevelControllerName(),   requestedHighLevelControllerState);
//      factory.addRequestedTransition(standReadyState.getHighLevelControllerName(),              calibrationControllerState.getHighLevelControllerName(), requestedHighLevelControllerState);
//      factory.addRequestedTransition(standReadyState.getHighLevelControllerName(),              standPrepControllerState.getHighLevelControllerName(),   requestedHighLevelControllerState);

      // Stand transition to manipulation
      factory.addTransition(standTransitionState.getHighLevelControllerName(),
                            new StateTransition<>(manipulationState.getHighLevelControllerName(), new StateTransitionCondition()
                            {
                               @Override
                               public boolean testCondition(double timeInCurrentState)
                               {
                                  return standTransitionState.isDone(timeInCurrentState);
                               }

                               @Override
                               public boolean performOnEntry()
                               {
                                  return false;
                               }
                            }));

      // Build state machine
      controllerStateMachine = factory.build(robotModel.getHighLevelControllerParameters().getDefaultInitialControllerState());

      // Attach registries
      registry.addChild(standPrepControllerState.getYoRegistry());
      registry.addChild(standReadyState.getYoRegistry());
      registry.addChild(standTransitionState.getYoRegistry());
      registry.addChild(calibrationControllerState.getYoRegistry());

      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();

      new DefaultParameterReader().readParametersInRegistry(registry);

      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), graphicsListRegistry);
      yoVariableServer.start();

      robotConfigurationDataPublisher.initialize();

      outputWriter = new ValkyrieUpperBodyOutputWriter(controlledOneDoFJoints, jointDesiredOutputList, effortJointHandles);
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      wallTimeProvider.setTimestamp(rosTime);
      yoTime.set(Conversions.nanosecondsToSeconds(monotonicTimeProvider.getTimestamp()));

      /* Perform state estimation */
      stateEstimator.update();

      /* Run state machine */
      controllerStateMachine.doActionAndTransition();
      jointDesiredOutputList.overwriteWith(controllerStateMachine.getCurrentState().getOutputForLowLevelController());

      /* Write desireds */
      outputWriter.write();

      /* Update YoVariable server */
      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
      robotConfigurationDataPublisher.write();
   }

   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.UPPER_BODY);

      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      List<GroupParameter<JointDesiredBehaviorReadOnly>> desiredJointBehaviors = highLevelControllerParameters.getDesiredJointBehaviors(STAND_PREP_STATE);

      if (desiredJointBehaviors != null)
      {
         for (int i = 0; i < desiredJointBehaviors.size(); i++)
         {
            System.out.println(desiredJointBehaviors.get(i).getGroupName());
            List<String> memberNames = desiredJointBehaviors.get(i).getMemberNames();
            for (int j = 0; j < memberNames.size(); j++)
            {
               System.out.println("  " + memberNames.get(j));
            }
         }
      }
   }
}
