package us.ihmc.valkyrieRosControl.impedance;

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
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
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
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrieRosControl.ValkyrieTorqueOffsetPrinter;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

public class ValkyrieWholeBodyImpedanceController extends IHMCWholeRobotControlJavaBridge
{
   static final List<String> allJoints = new ArrayList<>();
   static final List<String> torqueJoints = new ArrayList<>();
   static final List<String> impedanceJoints = new ArrayList<>();

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
   private final Map<String, AlphaFilteredYoVariable> nameToFilteredJointVelocity = new HashMap<>();
   private final ValkyrieImpedanceOutputWriter outputWriter;

   private final RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "valkyrie_whole_body_impedance_controller");
   private final EffortJointHandle[] effortHandles = new EffortJointHandle[allJoints.size()];
   private final JointImpedanceHandle[] impedanceHandles = new JointImpedanceHandle[impedanceJoints.size()];

   private final Map<String, EffortJointHandle> nameToEffortHandleMap = new HashMap<>();
   private final Map<String, JointImpedanceHandle> nameToImpedanceHandleMap = new HashMap<>();
   private final Map<String, JointspacePositionControllerState.OneDoFJointManager> nameToJointManagerMap = new HashMap<>();

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
      jointHome = createJointHomeConfiguration(robotModel.getJointMap());
      outputWriter = new ValkyrieImpedanceOutputWriter(fullRobotModel, nameToJointManagerMap, nameToEffortHandleMap, nameToImpedanceHandleMap);

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

         double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(25.0, robotModel.getControllerDT());
         AlphaFilteredYoVariable filteredVelocity = new AlphaFilteredYoVariable("qd_filt_" + controlledOneDoFJoints[i].getName(), registry, alphaVelocity);
         nameToFilteredJointVelocity.put(controlledOneDoFJoints[i].getName(), filteredVelocity);
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
      outputWriter.write();

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
      robotConfigurationDataPublisher.write();
   }

   private void read()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(allJoints.get(i));
         AlphaFilteredYoVariable filteredVelocity = nameToFilteredJointVelocity.get(allJoints.get(i));
         filteredVelocity.update(effortHandles[i].getVelocity());

         joint.setQ(effortHandles[i].getPosition());
         joint.setQd(filteredVelocity.getDoubleValue());
      }

      fullRobotModel.getRootBody().updateFramesRecursively();
   }

   private void doControl()
   {
      if (commandInputManager.isNewCommandAvailable(WholeBodyJointspaceTrajectoryCommand.class))
      {
         WholeBodyJointspaceTrajectoryCommand command = commandInputManager.pollNewestCommand(WholeBodyJointspaceTrajectoryCommand.class);

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

   static TObjectDoubleHashMap<String> createJointHomeConfiguration(ValkyrieJointMap jointMap)
   {
      TObjectDoubleHashMap<String> jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);

      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.75);
      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0);
      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), -0.1);

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), robotSide.negateIfRightSide(0.087));
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.6);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 1.3);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.7);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), robotSide.negateIfRightSide(-0.087));

         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.4);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-1.0));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.1);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), robotSide.negateIfRightSide(-1.3));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 1.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      }

      return jointHomeConfiguration;
   }

}
