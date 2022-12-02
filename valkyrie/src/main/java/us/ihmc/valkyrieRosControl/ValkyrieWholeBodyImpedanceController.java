package us.ihmc.valkyrieRosControl;

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
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.JointImpedanceHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

public class ValkyrieWholeBodyImpedanceController extends IHMCWholeRobotControlJavaBridge
{
   private static final String[] allJoints;
   private static final String[] impedanceJoints;

   static
   {
      List<String> allJointsList = new ArrayList<>();
      List<String> impedanceJointsList = new ArrayList<>();

      // Arms
      allJointsList.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
      allJointsList.addAll(Arrays.asList("rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"));

      // Torso
      allJointsList.addAll(Arrays.asList("torsoYaw", "torsoPitch", "torsoRoll"));

      // Legs
      allJointsList.addAll(Arrays.asList("leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"));
      allJointsList.addAll(Arrays.asList("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"));

      allJoints = allJointsList.toArray(new String[0]);

      for (String joint : allJointsList)
      {
         if (!joint.contains("Ankle"))
         {
            impedanceJointsList.add(joint);
         }
      }

      impedanceJoints = impedanceJointsList.toArray(new String[0]);
   }

   private boolean firstTick = true;

   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM);
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private final YoDouble yoTime = new YoDouble("yoTime", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] controlledOneDoFJoints;

   private final RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "valkyrie_whole_body_impedance_controller");
   private final EffortJointHandle[] effortHandles = new EffortJointHandle[allJoints.length];
   private final JointImpedanceHandle[] impedanceHandles = new JointImpedanceHandle[impedanceJoints.length];

   private final Map<String, EffortJointHandle> nameToEffortHandleMap = new HashMap<>();
   private final Map<String, JointImpedanceHandle> nameToImpedanceHandleMap = new HashMap<>();

   private final YoDouble masterGain = new YoDouble("masterGain", registry);
   private final YoDouble desiredJointStiffness = new YoDouble("desiredJointStiffness", registry);
   private final YoDouble desiredJointDamping = new YoDouble("desiredJointDamping", registry);

   private final JointspacePositionControllerState positionControllerState;
   private final YoLowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;

   private YoVariableServer yoVariableServer;

   public ValkyrieWholeBodyImpedanceController()
   {
      fullRobotModel = robotModel.createFullRobotModel();

      JointBasics[] jointsToIgnore = AvatarControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());
      JointBasics[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);

      desiredJointStiffness.set(200.0);
      desiredJointDamping.set(35.0);

      CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());
      ROS2Topic<?> inputTopic = ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName());
      ROS2Topic<?> outputTopic = ROS2Tools.getControllerOutputTopic(robotModel.getSimpleRobotName());
      new ControllerNetworkSubscriber(inputTopic, commandInputManager, outputTopic, statusMessageOutputManager, ros2Node);

      jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);
      positionControllerState = new JointspacePositionControllerState(HighLevelControllerName.CUSTOM1,
                                                                      commandInputManager,
                                                                      statusMessageOutputManager,
                                                                      controlledOneDoFJoints,
                                                                      yoTime,
                                                                      robotModel.getHighLevelControllerParameters(), jointDesiredOutputList);

      for (int i = 0; i < allJoints.length; i++)
      {
         nameToEffortHandleMap.put(allJoints[i], effortHandles[i]);
      }

      for (int i = 0; i < impedanceJoints.length; i++)
      {
         nameToImpedanceHandleMap.put(impedanceJoints[i], impedanceHandles[i]);
      }

      ros2Node.spin();
   }

   @Override
   protected void init()
   {
      for (int i = 0; i < allJoints.length; i++)
      {
         effortHandles[i] = createEffortJointHandle(allJoints[i]);
      }

      for (int i = 0; i < impedanceJoints.length; i++)
      {
         impedanceHandles[i] = createJointImpedanceHandle(impedanceJoints[i]);
      }

      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();

      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getRootBody(), graphicsListRegistry);
      yoVariableServer.start();
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
   }

   private void read()
   {
      for (int i = 0; i < allJoints.length; i++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(allJoints[i]);
         joint.setQ(effortHandles[i].getPosition());
         joint.setQd(effortHandles[i].getVelocity());
      }

      fullRobotModel.getRootBody().updateFramesRecursively();
   }

   private void doControl()
   {
      if (firstTick)
      {
         positionControllerState.onEntry();
         firstTick = false;
      }

      positionControllerState.doAction(yoTime.getDoubleValue());
   }

   private void write()
   {
      for (int i = 0; i < impedanceJoints.length; i++)
      {
         String jointName = impedanceJoints[i];
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointName);
         JointImpedanceHandle impedanceHandle = impedanceHandles[i];

         impedanceHandle.setStiffness(masterGain.getDoubleValue() * desiredJointStiffness.getDoubleValue());
         impedanceHandle.setDamping(masterGain.getDoubleValue() * desiredJointDamping.getDoubleValue());

         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);

         double desiredPosition = jointDesiredOutput.getDesiredPosition();
         double desiredVelocity = jointDesiredOutput.getDesiredVelocity();

         double epsilonNearLimit = Math.toRadians(3.5);
         double maxVelocity = 4.0;

         /* Clamp desired position */
         desiredPosition = EuclidCoreTools.clamp(desiredPosition, joint.getJointLimitLower(), joint.getJointLimitUpper());

         /* Scale down velocities near limits and clamp overall velocity */
         if (EuclidCoreTools.epsilonEquals(joint.getJointLimitLower(), desiredPosition, epsilonNearLimit) && desiredVelocity < 0.0)
         {
            double scale = (desiredPosition - joint.getJointLimitLower()) / epsilonNearLimit;
            desiredVelocity = scale * desiredVelocity;
         }
         else if (EuclidCoreTools.epsilonEquals(joint.getJointLimitUpper(), desiredPosition, epsilonNearLimit) && desiredVelocity > 0.0)
         {
            double scale = (joint.getJointLimitUpper() - desiredPosition) / epsilonNearLimit;
            desiredVelocity = scale * desiredVelocity;
         }

         desiredVelocity = EuclidCoreTools.clamp(desiredVelocity, maxVelocity);

         impedanceHandle.setPosition(desiredPosition);
         impedanceHandle.setVelocity(desiredVelocity);
      }
   }
}
