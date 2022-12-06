package us.ihmc.valkyrieRosControl;

import gnu.trove.impl.Constants;
import gnu.trove.map.hash.TIntIntHashMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyJointspaceTrajectoryCommand;
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
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.JointImpedanceHandle;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

public class ValkyrieWholeBodyImpedanceController extends IHMCWholeRobotControlJavaBridge
{
   private static final List<String> allJoints = new ArrayList<>();
   private static final List<String> torqueJoints = new ArrayList<>();
   private static final List<String> impedanceJoints = new ArrayList<>();
   private static final Map<String, Double> torqueOffsets = ValkyrieTorqueOffsetPrinter.loadTorqueOffsetsFromFile();

   static
   {
      // Arms
      allJoints.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
      allJoints.addAll(Arrays.asList("rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"));

      // Torso
      allJoints.addAll(Arrays.asList("torsoYaw", "torsoPitch", "torsoRoll"));

      // Legs
      allJoints.addAll(Arrays.asList("leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"));
      allJoints.addAll(Arrays.asList("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"));

      torqueJoints.add("torsoPitch");
      torqueJoints.add("torsoRoll");
      torqueJoints.add("leftAnklePitch");
      torqueJoints.add("leftAnkleRoll");
      torqueJoints.add("rightAnklePitch");
      torqueJoints.add("rightAnkleRoll");

      for (String joint : allJoints)
      {
         if (!torqueJoints.contains(joint))
         {
            impedanceJoints.add(joint);
         }
      }

      if (torqueOffsets == null)
      {
         throw new RuntimeException("Torque offsets file could not load");
      }
   }

   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM);
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private final YoDouble yoTime = new YoDouble("yoTime", registry);
   private final RobotConfigurationDataPublisher robotConfigurationDataPublisher;

   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final TObjectDoubleHashMap<String> jointHome;

   private final RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "valkyrie_whole_body_impedance_controller");
   private final EffortJointHandle[] effortHandles = new EffortJointHandle[allJoints.size()];
   private final JointImpedanceHandle[] impedanceHandles = new JointImpedanceHandle[impedanceJoints.size()];

   private final Map<String, EffortJointHandle> nameToEffortHandleMap = new HashMap<>();
   private final Map<String, JointImpedanceHandle> nameToImpedanceHandleMap = new HashMap<>();
   private final Map<String, JointspacePositionControllerState.OneDoFJointManager> nameToJointManagerMap = new HashMap<>();

   /* For joints with impedance API */
   private final YoDouble masterGain = new YoDouble("masterGain", registry);
   private final YoDouble desiredJointStiffness = new YoDouble("desiredJointStiffness", registry);
   private final YoDouble desiredJointDamping = new YoDouble("desiredJointDamping", registry);

   /* For joints without impedance API */
   private final YoDouble highLevelMasterGain = new YoDouble("highLevelMasterGain", registry);
   private final YoDouble highLevelJointStiffness = new YoDouble("highLevelJointStiffness", registry);
   private final YoDouble highLevelJointDamping = new YoDouble("highLevelJointDamping", registry);

   private final YoLowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;
   private final JointspacePositionControllerState.OneDoFJointManager[] jointManagers;

   private final TIntIntHashMap hashCodeToJointIndexMap = new TIntIntHashMap(40, Constants.DEFAULT_LOAD_FACTOR, -1, -1);
   private final CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

   private YoVariableServer yoVariableServer;

   public ValkyrieWholeBodyImpedanceController()
   {
      fullRobotModel = robotModel.createFullRobotModel();

      JointBasics[] jointsToIgnore = AvatarControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());
      JointBasics[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      jointHome = robotModel.getWalkingControllerParameters().getOrCreateJointHomeConfiguration();

      masterGain.set(0.15);
      desiredJointStiffness.set(200.0);
      desiredJointDamping.set(35.0);

      highLevelMasterGain.set(0.6);
      highLevelJointStiffness.set(35.0);
      highLevelJointDamping.set(6.0);

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

      jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);
      jointManagers = new JointspacePositionControllerState.OneDoFJointManager[controlledOneDoFJoints.length];
      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         jointManagers[i] = new JointspacePositionControllerState.OneDoFJointManager(controlledOneDoFJoints[i], yoTime, registry);
         nameToJointManagerMap.put(controlledOneDoFJoints[i].getName(), jointManagers[i]);
         hashCodeToJointIndexMap.put(controlledOneDoFJoints[i].hashCode(), i);
      }

      new DefaultParameterReader().readParametersInRegistry(registry);
      ros2Node.spin();
   }

   @Override
   protected void init()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         effortHandles[i] = createEffortJointHandle(allJoints.get(i));
         nameToEffortHandleMap.put(allJoints.get(i), effortHandles[i]);
      }

      for (int i = 0; i < impedanceJoints.size(); i++)
      {
         impedanceHandles[i] = createJointImpedanceHandle(impedanceJoints.get(i));
         nameToImpedanceHandleMap.put(impedanceJoints.get(i), impedanceHandles[i]);
      }

      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         jointManagers[i].holdPosition(jointHome.get(controlledOneDoFJoints[i].getName()));
      }

      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getRootBody(), graphicsListRegistry);
      yoVariableServer.start();

      robotConfigurationDataPublisher.initialize();
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      yoTime.set(Conversions.nanosecondsToSeconds(monotonicTimeProvider.getTimestamp()));

      /* Perform state estimation */
      read();

      /* Process incoming trajectories */
      doControl();

      /* Write to effort and impedance handles */
      write();

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
      robotConfigurationDataPublisher.write();
   }

   private void read()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(allJoints.get(i));
         joint.setQ(effortHandles[i].getPosition());
         joint.setQd(effortHandles[i].getVelocity());
      }

      fullRobotModel.getRootBody().updateFramesRecursively();
   }

   private void doControl()
   {
      if (commandInputManager.isNewCommandAvailable(WholeBodyJointspaceTrajectoryCommand.class))
      {
         WholeBodyJointspaceTrajectoryCommand command = commandInputManager.pollNewestCommand(WholeBodyJointspaceTrajectoryCommand.class);
         LogTools.info("Received message " + WholeBodyJointspaceTrajectoryCommand.class.getName());

         for (int commandIdx = 0; commandIdx < command.getNumberOfJoints(); commandIdx++)
         {
            int jointHashCode = command.getJointHashCode(commandIdx);
            OneDoFJointTrajectoryCommand jointTrajectory = command.getJointTrajectoryPointList(commandIdx);
            int jointIndex = hashCodeToJointIndexMap.get(jointHashCode);

            if (jointIndex == -1)
            {
               LogTools.warn("Joint not supported, hash-code: {}", jointHashCode);
               continue;
            }

            jointManagers[jointIndex].handleTrajectoryCommand(jointTrajectory, command);
         }
      }

      for (int i = 0; i < jointManagers.length; i++)
      {
         /* Sets position/velocity of PD controlled joints */
         jointManagers[i].doControl(jointDesiredOutputList.getJointDesiredOutput(controlledOneDoFJoints[i]));
      }
   }

   private void write()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         String jointName = allJoints.get(i);
         JointspacePositionControllerState.OneDoFJointManager jointManager = nameToJointManagerMap.get(jointName);
         EffortJointHandle effortJointHandle = nameToEffortHandleMap.get(jointName);
         double torqueOffset = torqueOffsets.get(jointName);

         if (impedanceJoints.contains(jointName))
         {
            JointImpedanceHandle impedanceHandle = nameToImpedanceHandleMap.get(jointName);

            impedanceHandle.setStiffness(masterGain.getDoubleValue() * desiredJointStiffness.getDoubleValue());
            impedanceHandle.setDamping(masterGain.getDoubleValue() * desiredJointDamping.getDoubleValue());
            impedanceHandle.setPosition(jointManager.getJointDesiredPosition());
            impedanceHandle.setVelocity(jointManager.getJointDesiredVelocity());
            effortJointHandle.setDesiredEffort(-torqueOffset);
         }
         else if (torqueJoints.contains(jointName))
         {
            OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointName);
            double q = joint.getQ();
            double qd = joint.getQd();
            double qDes = jointManager.getJointDesiredPosition();
            double qdDes = jointManager.getJointDesiredVelocity();

            double kp = highLevelMasterGain.getDoubleValue() * highLevelJointStiffness.getDoubleValue();
            double kd = highLevelMasterGain.getDoubleValue() * highLevelJointDamping.getDoubleValue();
            double effort = -torqueOffset + kp * (qDes - q) + kd * (qdDes - qd);
            effortJointHandle.setDesiredEffort(effort);
         }
         else
         {
            throw new RuntimeException("Joint " + jointName + " is not registered with either the impedance or torque lists");
         }
      }

//      for (int i = 0; i < impedanceJoints.length; i++)
//      {
//         String jointName = impedanceJoints[i];
//         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointName);
//         JointImpedanceHandle impedanceHandle = impedanceHandles[i];
//
//         impedanceHandle.setStiffness(masterGain.getDoubleValue() * desiredJointStiffness.getDoubleValue());
//         impedanceHandle.setDamping(masterGain.getDoubleValue() * desiredJointDamping.getDoubleValue());
//
//         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
//
//         double desiredPosition = jointDesiredOutput.getDesiredPosition();
//         double desiredVelocity = jointDesiredOutput.getDesiredVelocity();
//
//         double epsilonNearLimit = Math.toRadians(3.5);
//         double maxVelocity = 4.0;
//
//         /* Clamp desired position */
//         desiredPosition = EuclidCoreTools.clamp(desiredPosition, joint.getJointLimitLower(), joint.getJointLimitUpper());
//
//         /* Scale down velocities near limits and clamp overall velocity */
//         if (EuclidCoreTools.epsilonEquals(joint.getJointLimitLower(), desiredPosition, epsilonNearLimit) && desiredVelocity < 0.0)
//         {
//            double scale = (desiredPosition - joint.getJointLimitLower()) / epsilonNearLimit;
//            desiredVelocity = scale * desiredVelocity;
//         }
//         else if (EuclidCoreTools.epsilonEquals(joint.getJointLimitUpper(), desiredPosition, epsilonNearLimit) && desiredVelocity > 0.0)
//         {
//            double scale = (joint.getJointLimitUpper() - desiredPosition) / epsilonNearLimit;
//            desiredVelocity = scale * desiredVelocity;
//         }
//
//         desiredVelocity = EuclidCoreTools.clamp(desiredVelocity, maxVelocity);
//
//         impedanceHandle.setPosition(desiredPosition);
//         impedanceHandle.setVelocity(desiredVelocity);
//      }
//
//      writeTorqueOffsets();
   }

//   private void writeTorqueOffsets()
//   {
//      if (torqueOffsets == null)
//         return;
//
//      for (int i = 0; i < allJoints.length; i++)
//      {
//         String joint = allJoints[i];
//         Double offset = torqueOffsets.get(joint);
//
//         if (offset != null)
//         {
//            effortHandles[i].setDesiredEffort(-offset);
//         }
//      }
//   }
}
