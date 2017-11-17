package us.ihmc.valkyrieRosControl;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.STAND_PREP_STATE;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.STAND_READY;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.STAND_TRANSITION_STATE;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

import us.ihmc.affinity.Affinity;
import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.*;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.fingers.ValkyrieHandStateCommunicator;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithTorqueOffsets;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRobotControlElementCoordinator;
import us.ihmc.wholeBodyController.concurrent.SynchronousMultiThreadedRobotController;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;

public class ValkyrieRosControlController extends IHMCWholeRobotControlJavaBridge
{
   private static final String[] allValkyrieJoints = {
		   "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
         "torsoYaw", "torsoPitch", "torsoRoll",
         "lowerNeckPitch", "neckYaw", "upperNeckPitch",
         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch",
         "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch",
         "leftForearmYaw", "leftWristRoll", "leftWristPitch", 
         "rightForearmYaw", "rightWristRoll", "rightWristPitch",
         "leftIndexFingerPitch1", "leftMiddleFingerPitch1", "leftPinkyPitch1", "leftThumbRoll", "leftThumbPitch1", "leftThumbPitch2",
         "rightIndexFingerPitch1", "rightMiddleFingerPitch1", "rightPinkyPitch1", "rightThumbRoll", "rightThumbPitch1", "rightThumbPitch2"
         };

   private static final String[] torqueControlledJoints = {
         "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
         "torsoYaw", "torsoPitch", "torsoRoll",
         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch",
         "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch",
         "leftForearmYaw", "leftWristRoll", "leftWristPitch",
         "rightForearmYaw", "rightWristRoll", "rightWristPitch",
         "leftIndexFingerPitch1", "leftMiddleFingerPitch1", "leftPinkyPitch1", "leftThumbRoll", "leftThumbPitch1", "leftThumbPitch2",
         "rightIndexFingerPitch1", "rightMiddleFingerPitch1", "rightPinkyPitch1", "rightThumbRoll", "rightThumbPitch1", "rightThumbPitch2"
         };

   private static final String[] positionControlledJoints = {
         "lowerNeckPitch", "neckYaw", "upperNeckPitch",
   };

   public static final boolean USE_YOVARIABLE_DESIREDS = true;
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

   private static final WalkingProvider walkingProvider = WalkingProvider.DATA_PRODUCER;

   public static final boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = true;
   private static final boolean DO_SLOW_INTEGRATION_FOR_TORQUE_OFFSET = true;

   private MultiThreadedRobotControlElementCoordinator robotController;

   private final SettableTimestampProvider timestampProvider = new SettableTimestampProvider();

   private boolean firstTick = true;

   private final ValkyrieAffinity valkyrieAffinity;
   private boolean isGazebo;

   public ValkyrieRosControlController()
   {
      processEnvironmentVariables();
      valkyrieAffinity = new ValkyrieAffinity(!isGazebo);
   }

   private ValkyrieCalibrationControllerStateFactory calibrationStateFactory = null;

   private HighLevelHumanoidControllerFactory createHighLevelControllerFactory(ValkyrieRobotModel robotModel, PacketCommunicator packetCommunicator,
                                                                               DRCRobotSensorInformation sensorInformation)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();

      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory, feetForceSensorNames,
                                                                                                    feetContactSensorNames, wristForceSensorNames,
                                                                                                    highLevelControllerParameters, walkingControllerParameters,
                                                                                                    capturePointPlannerParameters);

      PeriodicRealtimeThreadScheduler networkSubscriberScheduler = new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY);
      controllerFactory.createControllerNetworkSubscriber(networkSubscriberScheduler, packetCommunicator);

      // setup states
      controllerFactory.setInitialState(highLevelControllerParameters.getDefaultInitialControllerState());
      controllerFactory.useDefaultStandPrepControlState();
      controllerFactory.useDefaultStandReadyControlState();
      controllerFactory.useDefaultStandTransitionControlState();
      controllerFactory.useDefaultWalkingControlState();

      ValkyrieTorqueOffsetPrinter valkyrieTorqueOffsetPrinter = new ValkyrieTorqueOffsetPrinter();
      valkyrieTorqueOffsetPrinter.setRobotName(robotModel.getFullRobotName());
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

      controllerFactory.addRequestableTransition(WALKING, STAND_PREP_STATE);
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
         robotModel = new ValkyrieRobotModel(RobotTarget.GAZEBO, true);
      }
      else
      {
         robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, true);
      }

      ValkyrieSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create network servers/clients
       */
      PacketCommunicator controllerPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT,
                                                                                                             new IHMCCommunicationKryoNetClassList());
      PeriodicRealtimeThreadSchedulerFactory yoVariableServerScheduler = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.LOGGER_PRIORITY);
      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      LogSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), yoVariableServerScheduler, logModelProvider, logSettings, estimatorDT);
      HumanoidGlobalDataProducer dataProducer = new HumanoidGlobalDataProducer(controllerPacketCommunicator);

      /*
       * Create sensors
       */

      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(timestampProvider, stateEstimatorParameters,
                                                                                                            effortJointHandles, positionJointHandles,
                                                                                                            jointStateHandles, imuHandles,
                                                                                                            forceTorqueSensorHandles, jointMap, sensorInformation);

      /*
       * Create controllers
       */
      HighLevelHumanoidControllerFactory controllerFactory = createHighLevelControllerFactory(robotModel, controllerPacketCommunicator, sensorInformation);
      CommandInputManager commandInputManager = controllerFactory.getCommandInputManager();
      StatusMessageOutputManager statusOutputManager = controllerFactory.getStatusOutputManager();

      /*
       * Create output writer
       */
      ValkyrieRosControlLowLevelOutputWriter valkyrieLowLevelOutputWriter = new ValkyrieRosControlLowLevelOutputWriter();

      DRCOutputProcessor drcOutputWriter = null;
      if (DO_SLOW_INTEGRATION_FOR_TORQUE_OFFSET)
      {
         double controllerDT = robotModel.getControllerDT();
         DRCOutputProcessorWithTorqueOffsets drcOutputWriterWithTorqueOffsets = new DRCOutputProcessorWithTorqueOffsets(drcOutputWriter, controllerDT);
         drcOutputWriter = drcOutputWriterWithTorqueOffsets;
      }

      PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = null;
      externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(null);
      dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

      /*
       * Build controller
       */
      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
      RobotContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      PeriodicRealtimeThreadScheduler estimatorScheduler = new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY);
      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(sensorInformation, contactPointParameters, robotModel, stateEstimatorParameters, sensorReaderFactory,
                                                                  threadDataSynchronizer, estimatorScheduler, dataProducer, valkyrieLowLevelOutputWriter,
                                                                  yoVariableServer, gravity);
      estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);

      PeriodicRealtimeThreadScheduler handStateCommunicatorScheduler = new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.HAND_COMMUNICATOR_PRIORITY);
      ValkyrieHandStateCommunicator handStateCommunicator = new ValkyrieHandStateCommunicator(threadDataSynchronizer.getEstimatorFullRobotModel(), robotModel.getHandModel(), controllerPacketCommunicator, handStateCommunicatorScheduler);
      handStateCommunicator.start();
      estimatorThread.addRobotController(handStateCommunicator);

      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, sensorInformation, controllerFactory, threadDataSynchronizer, drcOutputWriter,
                                                                     dataProducer, yoVariableServer, gravity, estimatorDT);

      ValkyrieCalibrationControllerState calibrationControllerState = calibrationStateFactory.getCalibrationControllerState();
      calibrationControllerState.attachForceSensorCalibrationModule(estimatorThread.getForceSensorCalibrationModule());

      sensorReaderFactory.attachControllerAPI(commandInputManager, statusOutputManager);
      sensorReaderFactory.attachJointTorqueOffsetEstimator(calibrationControllerState.getJointTorqueOffsetEstimatorController());
      sensorReaderFactory.setupLowLevelControlWithPacketCommunicator(controllerPacketCommunicator);

      /*
       * Connect all servers
       */
      try
      {
         controllerPacketCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      yoVariableServer.start();

      if (isGazebo)
      {
         PrintTools.info(ValkyrieRosControlController.class, "Running with blocking synchronous execution between estimator and controller");
         SynchronousMultiThreadedRobotController coordinator = new SynchronousMultiThreadedRobotController(estimatorThread, timestampProvider);
         coordinator.addController(controllerThread, (int) (robotModel.getControllerDT() / robotModel.getEstimatorDT()));

         robotController = coordinator;
      }
      else
      {
         PrintTools.info(ValkyrieRosControlController.class, "Running with multi-threaded RT threads for estimator and controller");
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
   protected void doControl(long time, long duration)
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

      timestampProvider.setTimestamp(time);
      robotController.read();
   }
}
