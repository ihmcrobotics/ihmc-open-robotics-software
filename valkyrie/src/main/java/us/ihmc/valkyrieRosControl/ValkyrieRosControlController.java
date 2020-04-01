package us.ihmc.valkyrieRosControl;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.EXIT_WALKING;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.STAND_PREP_STATE;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.STAND_READY;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.STAND_TRANSITION_STATE;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.affinity.Affinity;
import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.ForceTorqueSensorHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.IMUHandle;
import us.ihmc.rosControl.wholeRobot.JointStateHandle;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.fingers.ValkyrieHandStateCommunicator;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithStateChangeSmoother;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRobotControlElementCoordinator;
import us.ihmc.wholeBodyController.concurrent.SynchronousMultiThreadedRobotController;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieRosControlController extends IHMCWholeRobotControlJavaBridge
{
   // Note: keep committed as DEFAULT, only change locally if needed
   public static final ValkyrieRobotVersion VERSION = ValkyrieRobotVersion.fromEnvironment();

   public static final boolean ENABLE_FINGER_JOINTS = VERSION.hasFingers();
   public static final boolean HAS_LIGHTER_BACKPACK = true;
   public static final boolean LOG_SECONDARY_HIGH_LEVEL_STATES = false;

   private static final String[] torqueControlledJoints;
   static
   {
      List<String> jointList = new ArrayList<>();
      jointList.addAll(Arrays.asList("leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"));
      jointList.addAll(Arrays.asList("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"));
      jointList.addAll(Arrays.asList("torsoYaw", "torsoPitch", "torsoRoll"));

      switch (VERSION)
      {
         case DEFAULT:
            jointList.addAll(Arrays.asList("leftIndexFingerMotorPitch1",
                                           "leftMiddleFingerMotorPitch1",
                                           "leftPinkyMotorPitch1",
                                           "leftThumbMotorRoll",
                                           "leftThumbMotorPitch1",
                                           "leftThumbMotorPitch2"));
            jointList.addAll(Arrays.asList("rightIndexFingerMotorPitch1",
                                           "rightMiddleFingerMotorPitch1",
                                           "rightPinkyMotorPitch1",
                                           "rightThumbMotorRoll",
                                           "rightThumbMotorPitch1",
                                           "rightThumbMotorPitch2"));
         case FINGERLESS:
            jointList.addAll(Arrays.asList("leftForearmYaw", "leftWristRoll", "leftWristPitch"));
            jointList.addAll(Arrays.asList("rightForearmYaw", "rightWristRoll", "rightWristPitch"));
         case ARM_MASS_SIM:
            jointList.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
            jointList.addAll(Arrays.asList("rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"));
      }

      torqueControlledJoints = jointList.toArray(new String[0]);
   }

   private static final String[] positionControlledJoints = {"lowerNeckPitch", "neckYaw", "upperNeckPitch",};

   private static final String[] allValkyrieJoints;
   static
   {
      List<String> allJointsList = new ArrayList<>();
      Arrays.stream(torqueControlledJoints).forEach(allJointsList::add);
      Arrays.stream(positionControlledJoints).forEach(allJointsList::add);

      if (ENABLE_FINGER_JOINTS)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String prefix = robotSide.getCamelCaseName();
            allJointsList.addAll(Arrays.asList(prefix + "IndexFingerPitch1", prefix + "IndexFingerPitch2", prefix + "IndexFingerPitch3"));
            allJointsList.addAll(Arrays.asList(prefix + "MiddleFingerPitch1", prefix + "MiddleFingerPitch2", prefix + "MiddleFingerPitch3"));
            allJointsList.addAll(Arrays.asList(prefix + "PinkyPitch1", prefix + "PinkyPitch2", prefix + "PinkyPitch3"));
            allJointsList.addAll(Arrays.asList(prefix + "ThumbRoll", prefix + "ThumbPitch1", prefix + "ThumbPitch2", prefix + "ThumbPitch3"));
         }
      }
      allValkyrieJoints = allJointsList.toArray(new String[0]);
   }

   public static final boolean USE_STATE_CHANGE_TORQUE_SMOOTHER_PROCESSOR = true;
   public static final boolean USE_YOVARIABLE_DESIREDS = false;
   public static final boolean USE_USB_MICROSTRAIN_IMUS = false;
   public static final boolean USE_SWITCHABLE_FILTER_HOLDER_FOR_NON_USB_IMUS = false;
   public static final String[] readIMUs = USE_USB_MICROSTRAIN_IMUS ? new String[0] : new String[ValkyrieSensorInformation.imuSensorsToUse.length];

   static
   {
      if (!USE_USB_MICROSTRAIN_IMUS)
      {
         for (int i = 0; i < ValkyrieSensorInformation.imuSensorsToUse.length; i++)
         {
            readIMUs[i] = ValkyrieSensorInformation.imuSensorsToUse[i].replace("pelvis_", "").replace("torso_", "");
         }
      }
   }

   public static final String[] readForceTorqueSensors = {"leftFootSixAxis", "rightFootSixAxis"};
   public static final String[] forceTorqueSensorModelNames = {"leftAnkleRoll", "rightAnkleRoll"};

   public static final double gravity = 9.80665;

   public static final String VALKYRIE_IHMC_ROS_ESTIMATOR_NODE_NAME = "valkyrie_ihmc_state_estimator";
   public static final String VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME = ROS2Tools.HUMANOID_CONTROLLER.getNodeName("valkyrie");

   private static final WalkingProvider walkingProvider = WalkingProvider.DATA_PRODUCER;

   private MultiThreadedRobotControlElementCoordinator robotController;

   private final SettableTimestampProvider wallTimeProvider = new SettableTimestampProvider();
   private final TimestampProvider monotonicTimeProvider = () -> RealtimeThread.getCurrentMonotonicClockTime();

   private boolean firstTick = true;

   private final ValkyrieAffinity valkyrieAffinity;
   private boolean isGazebo;

   public ValkyrieRosControlController()
   {
      processEnvironmentVariables();
      valkyrieAffinity = new ValkyrieAffinity(!isGazebo);
   }

   private ValkyrieCalibrationControllerStateFactory calibrationStateFactory = null;

   private HighLevelHumanoidControllerFactory createHighLevelControllerFactory(ValkyrieRobotModel robotModel, RealtimeRos2Node realtimeRos2Node,
                                                                               HumanoidRobotSensorInformation sensorInformation)
   {
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
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();

      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    feetForceSensorNames,
                                                                                                    feetContactSensorNames,
                                                                                                    wristForceSensorNames,
                                                                                                    highLevelControllerParameters,
                                                                                                    walkingControllerParameters,
                                                                                                    capturePointPlannerParameters);

      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeRos2Node);

      // setup states
      controllerFactory.setInitialState(highLevelControllerParameters.getDefaultInitialControllerState());
      controllerFactory.useDefaultStandPrepControlState();
      controllerFactory.useDefaultStandReadyControlState();
      controllerFactory.useDefaultStandTransitionControlState();
      controllerFactory.useDefaultWalkingControlState();
      controllerFactory.useDefaultExitWalkingTransitionControlState(STAND_PREP_STATE);

      ValkyrieTorqueOffsetPrinter valkyrieTorqueOffsetPrinter = new ValkyrieTorqueOffsetPrinter();
      calibrationStateFactory = new ValkyrieCalibrationControllerStateFactory(valkyrieTorqueOffsetPrinter, robotModel.getCalibrationParameters());
      controllerFactory.addCustomControlState(calibrationStateFactory);

      // setup transitions
      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();

      HighLevelControllerName calibrationState = calibrationStateFactory.getStateEnum();
      controllerFactory.addRequestableTransition(calibrationState, STAND_PREP_STATE);
      controllerFactory.addFinishedTransition(calibrationState, STAND_PREP_STATE);

      controllerFactory.addFinishedTransition(STAND_PREP_STATE, STAND_READY);
      controllerFactory.addRequestableTransition(STAND_PREP_STATE, calibrationState);

      controllerFactory.addRequestableTransition(STAND_READY, STAND_TRANSITION_STATE);
      if (fallbackControllerState != STAND_READY)
         controllerFactory.addControllerFailureTransition(STAND_READY, fallbackControllerState);
      controllerFactory.addRequestableTransition(STAND_READY, calibrationState);
      controllerFactory.addRequestableTransition(STAND_READY, STAND_PREP_STATE);

      controllerFactory.addFinishedTransition(STAND_TRANSITION_STATE, WALKING);
      controllerFactory.addControllerFailureTransition(STAND_TRANSITION_STATE, fallbackControllerState);

      controllerFactory.addRequestableTransition(WALKING, EXIT_WALKING);
      controllerFactory.addFinishedTransition(EXIT_WALKING, STAND_PREP_STATE);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      if (walkingProvider == WalkingProvider.VELOCITY_HEADING_COMPONENT)
         controllerFactory.createComponentBasedFootstepDataMessageGenerator();
      if (USE_YOVARIABLE_DESIREDS)
         controllerFactory.createUserDesiredControllerCommandGenerator();

      return controllerFactory;
   }

   @Override
   protected void init()
   {
      LogTools.info("Valkyrie robot version: " + VERSION);
      /*
       * Create joints
       */

      HashSet<String> torqueControlledJointsSet = new HashSet<>(Arrays.asList(torqueControlledJoints));
      HashSet<String> positionControlledJointsSet = new HashSet<>(Arrays.asList(positionControlledJoints));

      HashMap<String, EffortJointHandle> effortJointHandles = new HashMap<>();
      HashMap<String, PositionJointHandle> positionJointHandles = new HashMap<>();
      HashMap<String, JointStateHandle> jointStateHandles = new HashMap<>();

      for (String joint : allValkyrieJoints)
      {
         if (torqueControlledJointsSet.contains(joint) && positionControlledJointsSet.contains(joint))
         {
            throw new RuntimeException("Joint cannot be both position controlled and torque controlled via ROS Control! Joint name: " + joint);
         }

         if (torqueControlledJointsSet.contains(joint))
         {
            effortJointHandles.put(joint, createEffortJointHandle(joint));
         }

         if (positionControlledJointsSet.contains(joint))
         {
            positionJointHandles.put(joint, createPositionJointHandle(joint));
         }

         if (!(torqueControlledJointsSet.contains(joint) || positionControlledJointsSet.contains(joint)))
         {
            jointStateHandles.put(joint, createJointStateHandle(joint));
         }
      }

      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      for (String imu : readIMUs)
      {
         if (USE_SWITCHABLE_FILTER_HOLDER_FOR_NON_USB_IMUS)
         {
            String complimentaryFilterHandleName = "CF" + imu;
            String kalmanFilterHandleName = "EF" + imu;
            imuHandles.put(complimentaryFilterHandleName, createIMUHandle(complimentaryFilterHandleName));
            imuHandles.put(kalmanFilterHandleName, createIMUHandle(kalmanFilterHandleName));
         }
         else
         {
            imuHandles.put(imu, createIMUHandle(imu));
         }
      }

      HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles = new HashMap<>();
      for (int i = 0; i < readForceTorqueSensors.length; i++)
      {
         String forceTorqueSensor = readForceTorqueSensors[i];
         String modelName = forceTorqueSensorModelNames[i];
         forceTorqueSensorHandles.put(modelName, createForceTorqueSensorHandle(forceTorqueSensor));
      }

      /*
       * Create registries
       */

      ValkyrieRobotModel robotModel;
      if (isGazebo)
      {
         robotModel = new ValkyrieRobotModel(RobotTarget.GAZEBO, VERSION);
      }
      else
      {
         robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, VERSION);
      }

      String robotName = robotModel.getSimpleRobotName();
      ValkyrieSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create network servers/clients
       */
      PeriodicRealtimeThreadSchedulerFactory realtimeThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY);
      RealtimeRos2Node estimatorRealtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS,
                                                                                    realtimeThreadFactory,
                                                                                    VALKYRIE_IHMC_ROS_ESTIMATOR_NODE_NAME);
      RealtimeRos2Node controllerRealtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS,
                                                                                     realtimeThreadFactory,
                                                                                     VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME);
      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);

      /*
       * Create sensors
       */

      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(wallTimeProvider,
                                                                                                            monotonicTimeProvider,
                                                                                                            stateEstimatorParameters,
                                                                                                            effortJointHandles,
                                                                                                            positionJointHandles,
                                                                                                            jointStateHandles,
                                                                                                            imuHandles,
                                                                                                            forceTorqueSensorHandles,
                                                                                                            jointMap,
                                                                                                            sensorInformation);

      /*
       * Create controllers
       */
      HighLevelHumanoidControllerFactory controllerFactory = createHighLevelControllerFactory(robotModel, controllerRealtimeRos2Node, sensorInformation);
      JointDesiredOutputWriter outputWriter = null;
      DRCOutputProcessor drcOutputProcessor = null;

      if (USE_STATE_CHANGE_TORQUE_SMOOTHER_PROCESSOR)
      {
         DRCOutputProcessorWithStateChangeSmoother outputSmoother = new DRCOutputProcessorWithStateChangeSmoother(drcOutputProcessor);
         controllerFactory.attachControllerStateChangedListener(outputSmoother.createControllerStateChangedListener());
         drcOutputProcessor = outputSmoother;
      }

      PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = null;
      externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(null, null);
      ROS2Tools.createCallbackSubscription(estimatorRealtimeRos2Node,
                                           StampedPosePacket.class,
                                           ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName),
                                           externalPelvisPoseSubscriber);

      /*
       * Build controller
       */
      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSimpleRobotName(),
                                                                  sensorInformation,
                                                                  contactPointParameters,
                                                                  robotModel,
                                                                  stateEstimatorParameters,
                                                                  sensorReaderFactory,
                                                                  threadDataSynchronizer,
                                                                  estimatorRealtimeRos2Node,
                                                                  externalPelvisPoseSubscriber,
                                                                  outputWriter,
                                                                  yoVariableServer,
                                                                  gravity);

      if (ENABLE_FINGER_JOINTS)
      {
         ValkyrieHandStateCommunicator handStateCommunicator = new ValkyrieHandStateCommunicator(robotName,
                                                                                                 threadDataSynchronizer.getEstimatorFullRobotModel(),
                                                                                                 robotModel.getHandModel(),
                                                                                                 estimatorRealtimeRos2Node);
         estimatorThread.addRobotController(handStateCommunicator);
      }

      DRCControllerThread controllerThread = new DRCControllerThread(robotModel.getSimpleRobotName(),
                                                                     robotModel,
                                                                     sensorInformation,
                                                                     controllerFactory,
                                                                     threadDataSynchronizer,
                                                                     drcOutputProcessor,
                                                                     controllerRealtimeRos2Node,
                                                                     yoVariableServer,
                                                                     gravity,
                                                                     estimatorDT);
      if (!LOG_SECONDARY_HIGH_LEVEL_STATES)
         detachSecondaryRegistries(controllerThread.getYoVariableRegistry());

      ValkyrieCalibrationControllerState calibrationControllerState = calibrationStateFactory.getCalibrationControllerState();
      calibrationControllerState.attachForceSensorCalibrationModule(estimatorThread.getForceSensorCalibrationModule());

      // Attaching listeners. Need to double check any possible threading issues here. Might be fine because we're using the StatusMessageOutputManager.
      sensorReaderFactory.attachControllerAPI(controllerFactory.getStatusOutputManager());
      sensorReaderFactory.attachJointTorqueOffsetEstimator(calibrationControllerState.getJointTorqueOffsetEstimatorController());
      sensorReaderFactory.setupLowLevelControlCommunication(robotName, estimatorRealtimeRos2Node);

      /*
       * Setup and start the JVM memory statistics
       */

      PeriodicRealtimeThreadSchedulerFactory schedulerFactory = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.JVM_STATISTICS_PRIORITY);
      JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer, schedulerFactory);
      jvmStatisticsGenerator.addVariablesToStatisticsGenerator(yoVariableServer);
      jvmStatisticsGenerator.start();

      /*
       * Connect all servers
       */
      estimatorRealtimeRos2Node.spin();
      controllerRealtimeRos2Node.spin();
      yoVariableServer.start();

      if (isGazebo)
      {
         LogTools.info("Running with blocking synchronous execution between estimator and controller");
         SynchronousMultiThreadedRobotController coordinator = new SynchronousMultiThreadedRobotController(estimatorThread, wallTimeProvider);
         coordinator.addController(controllerThread, (int) (robotModel.getControllerDT() / robotModel.getEstimatorDT()));

         robotController = coordinator;
      }
      else
      {
         LogTools.info("Running with multi-threaded RT threads for estimator and controller");
         MultiThreadedRealTimeRobotController coordinator = new MultiThreadedRealTimeRobotController(estimatorThread);
         if (valkyrieAffinity.setAffinity())
         {
            coordinator.addController(controllerThread, ValkyriePriorityParameters.CONTROLLER_PRIORITY, valkyrieAffinity.getControlThreadProcessor());
         }
         else
         {
            coordinator.addController(controllerThread, ValkyriePriorityParameters.CONTROLLER_PRIORITY, null);
         }

         robotController = coordinator;
      }

      robotController.start();
   }

   private void detachSecondaryRegistries(YoVariableRegistry drcControllerThreadRegistry)
   {
      YoVariableRegistry drcMomentumBasedControllerRegistry = findChild(drcControllerThreadRegistry, "DRCMomentumBasedController");
      YoVariableRegistry humanoidHighLevelControllerManagerRegistry = findChild(drcMomentumBasedControllerRegistry, "HumanoidHighLevelControllerManager");

      removeChild(humanoidHighLevelControllerManagerRegistry, "ValkyrieCalibrationControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "StandPrepControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "StandReadyControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "toWalkingSmoothTransitionControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "exitWalkingSmoothTransitionControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "LowLevelOneDoFJointDesiredDataHumanoidHighLevelControllerManager");
   }

   private static YoVariableRegistry findChild(YoVariableRegistry parent, String childName)
   {
      return parent.getChildren().stream().filter(child -> child.getName().equals(childName)).findFirst().get();
   }

   private static void removeChild(YoVariableRegistry parent, String nameOfChildToRemove)
   {
      parent.getChildren().remove(findChild(parent, nameOfChildToRemove));
   }

   private void processEnvironmentVariables()
   {
      String isGazeboEnvironmentVariable = System.getenv("IS_GAZEBO");
      isGazebo = false;
      if (isGazeboEnvironmentVariable != null)
      {
         switch (isGazeboEnvironmentVariable)
         {
            case "true":
               isGazebo = true;
               break;
            default:
               isGazebo = false;
               break;
         }
      }
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      if (firstTick)
      {
         if (valkyrieAffinity.setAffinity())
         {
            System.out.println("Setting estimator thread affinity to processor " + valkyrieAffinity.getEstimatorThreadProcessor().getId());
            Affinity.setAffinity(valkyrieAffinity.getEstimatorThreadProcessor());
         }
         firstTick = false;
      }

      wallTimeProvider.setTimestamp(rosTime);
      robotController.read();
   }
}
