package us.ihmc.valkyrieRosControl.upperBody;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.*;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrieRosControl.ValkyriePriorityParameters;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReader;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlSensorReaderFactory;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME;

public class ValkyrieUpperBodyController extends IHMCWholeRobotControlJavaBridge
{
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

   private static final ValkyrieRobotVersion VERSION = ValkyrieRobotVersion.UPPER_BODY;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, VERSION);
   private final FullHumanoidRobotModel fullRobotModel;
   private final ValkyrieUpperBodyStateEstimator stateEstimator;
   private final RealtimeROS2Node ros2Node;
   private final RobotConfigurationDataPublisher robotConfigurationDataPublisher;

   private final YoDouble yoTime = new YoDouble("yoTime", registry);
   private final SettableTimestampProvider wallTimeProvider = new SettableTimestampProvider();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private final YoLowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;

   private final HashMap<String, EffortJointHandle> effortJointHandles = new HashMap<>();
   private final HashMap<String, PositionJointHandle> positionJointHandles = new HashMap<>();
   private final HashMap<String, JointStateHandle> jointStateHandles = new HashMap<>();

   private YoVariableServer yoVariableServer;

   public ValkyrieUpperBodyController()
   {
      fullRobotModel = robotModel.createFullRobotModel();
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      PeriodicRealtimeThreadSchedulerFactory realtimeThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY);
      ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, realtimeThreadFactory, VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME);
      stateEstimator = new ValkyrieUpperBodyStateEstimator(fullRobotModel.getRootJoint(), oneDoFJoints, registry);

      jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(oneDoFJoints, registry);

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

      for (String joint : allValkyrieJoints)
      {
         if (torqueControlledJointsSet.contains(joint) && positionControlledJointsSet.contains(joint))
            throw new RuntimeException("Joint cannot be both position controlled and torque controlled via ROS Control! Joint name: " + joint);
         if (torqueControlledJointsSet.contains(joint))
            effortJointHandles.put(joint, createEffortJointHandle(joint));
         else if (positionControlledJointsSet.contains(joint))
            positionJointHandles.put(joint, createPositionJointHandle(joint));
         else
            jointStateHandles.put(joint, createJointStateHandle(joint));
      }

      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      for (String imu : ValkyrieRosControlController.readIMUs)
         imuHandles.put(imu, createIMUHandle(imu));

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

      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();

      new DefaultParameterReader().readParametersInRegistry(registry);
      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), new YoGraphicsListRegistry());
      yoVariableServer.start();

      robotConfigurationDataPublisher.initialize();
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      wallTimeProvider.setTimestamp(rosTime);
      yoTime.set(Conversions.nanosecondsToSeconds(monotonicTimeProvider.getTimestamp()));

      /* Perform state estimation */
      stateEstimator.update();

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
      robotConfigurationDataPublisher.write();
   }
}
