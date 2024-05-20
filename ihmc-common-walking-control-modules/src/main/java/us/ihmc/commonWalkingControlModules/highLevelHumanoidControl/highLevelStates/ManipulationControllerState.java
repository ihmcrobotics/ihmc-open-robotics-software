package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;
import us.ihmc.mecano.frames.MovingCenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ManipulationControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerName = HighLevelControllerName.WALKING;
   private static final double gravityZ = 9.81;

   public static final String weightRegistryName = "MomentumOptimizationSettings";
   public static final String jointspaceGainRegistryName = "JointspaceGains";
   public static final String rigidBodyGainRegistryName = "RigidBodyGains";
   public static final String comHeightGainRegistryName = "ComHeightGains";

   private final YoRegistry momentumRegistry = new YoRegistry(weightRegistryName);
   private final YoRegistry jointGainRegistry = new YoRegistry(jointspaceGainRegistryName);
   private final YoRegistry bodyGainRegistry = new YoRegistry(rigidBodyGainRegistryName);
   private final YoRegistry comHeightGainRegistry = new YoRegistry(comHeightGainRegistryName);
   private final Map<String, Vector3DReadOnly> taskspaceAngularWeightMap = new HashMap<>();
   private final Map<String, Vector3DReadOnly> taskspaceLinearWeightMap = new HashMap<>();

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);
   private final MomentumOptimizationSettings momentumOptimizationSettings;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();

   private final Map<String, PIDGainsReadOnly> jointspaceGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspaceOrientationGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspacePositionGainMap = new HashMap<>();
   private final Map<String, DoubleProvider> jointspaceWeightMap = new HashMap<>();
   private final Map<String, DoubleProvider> userModeWeightMap = new HashMap<>();

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final FullHumanoidRobotModel fullHumanoidRobotModel;

   private final HighLevelControllerParameters highLevelControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final MultiBodySystemBasics controllerSystem;
   private final ReferenceFrame centerOfMassFrame;
   private final WholeBodyControllerCore controllerCore;
   private SideDependentList<MovingReferenceFrame> handControlFrames = new SideDependentList<>();

   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private double controlDT;

   public ManipulationControllerState(CommandInputManager commandInputManager,
                                      StatusMessageOutputManager statusMessageOutputManager,
                                      double controlDT,
                                      HumanoidJointNameMap jointNameMap,
                                      HighLevelControllerParameters highLevelControllerParameters,
                                      WalkingControllerParameters walkingControllerParameters,
                                      FullHumanoidRobotModel fullRobotModel,
                                      JointBasics[] jointsToIgnore,
                                      YoDouble yoTime,
                                      YoGraphicsListRegistry graphicsListRegistry)
   {
      super(controllerName, highLevelControllerParameters, collectControllableJoints(fullRobotModel, jointsToIgnore));

      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.controlDT = controlDT;
      this.fullHumanoidRobotModel = fullRobotModel;

      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver();
      referenceFrameHashCodeResolver.put(fullRobotModel.getRootBody().getBodyFixedFrame());

      for (int i = 0; i < controlledJoints.length; i++)
      {
         referenceFrameHashCodeResolver.put(controlledJoints[i].getFrameBeforeJoint());
         referenceFrameHashCodeResolver.put(controlledJoints[i].getFrameAfterJoint());
         referenceFrameHashCodeResolver.put(controlledJoints[i].getSuccessor().getBodyFixedFrame());
      }

      commandInputManager.registerConversionHelper(new FrameMessageCommandConverter(referenceFrameHashCodeResolver));

      momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      ParameterTools.extractJointGainMap(walkingControllerParameters.getJointSpaceControlGains(), jointspaceGainMap, jointGainRegistry);
      ParameterTools.extract3DGainMap("Orientation",
                                      walkingControllerParameters.getTaskspaceOrientationControlGains(),
                                      taskspaceOrientationGainMap,
                                      bodyGainRegistry);
      ParameterTools.extract3DGainMap("Position", walkingControllerParameters.getTaskspacePositionControlGains(), taskspacePositionGainMap, bodyGainRegistry);
      ParameterTools.extractJointWeightMap("JointspaceWeight", momentumOptimizationSettings.getJointspaceWeights(), jointspaceWeightMap, momentumRegistry);
      ParameterTools.extractJointWeightMap("UserModeWeight", momentumOptimizationSettings.getUserModeWeights(), userModeWeightMap, momentumRegistry);
      ParameterTools.extract3DWeightMap("AngularWeight",
                                        momentumOptimizationSettings.getTaskspaceAngularWeights(),
                                        taskspaceAngularWeightMap,
                                        momentumRegistry);
      ParameterTools.extract3DWeightMap("LinearWeight", momentumOptimizationSettings.getTaskspaceLinearWeights(), taskspaceLinearWeightMap, momentumRegistry);

      String chestName = jointNameMap.getChestName();
      String headName = jointNameMap.getHeadName();

      controllerSystem = MultiBodySystemBasics.toMultiBodySystemBasics(fullHumanoidRobotModel.getRootBody());

      RigidBodyBasics elevator = controllerSystem.getRootBody();
      RigidBodyBasics chest = controllerSystem.findRigidBody(chestName);
      RigidBodyBasics head = controllerSystem.findRigidBody(headName);

      chestManager = createRigidBodyManager(chest, elevator, chest.getBodyFixedFrame(), elevator.getBodyFixedFrame(), elevator, yoTime, graphicsListRegistry);
      headManager = createRigidBodyManager(head, chest, head.getBodyFixedFrame(), chest.getBodyFixedFrame(), elevator, yoTime, graphicsListRegistry);

      privilegedConfigurationCommand.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = controllerSystem.findRigidBody(jointNameMap.getHandName(robotSide));

         RigidBodyTransform handControlFrameTransform = jointNameMap.getHandControlFrameToWristTransform(robotSide);
         if (handControlFrameTransform != null)
         {
            handControlFrames.put(robotSide,
                                  new FixedMovingReferenceFrame(robotSide.getCamelCaseName() + "HandControlFrame",
                                                                hand.getParentJoint().getFrameAfterJoint(),
                                                                handControlFrameTransform));
         }

         RigidBodyControlManager handManager = createRigidBodyManager(hand, chest, handControlFrames.get(robotSide), chest.getBodyFixedFrame(), elevator, yoTime, graphicsListRegistry);
         handManager.setDoPrepareForLocomotion(false);
         handManagers.put(robotSide, handManager);

         List<String> armJointNames = jointNameMap.getArmJointNamesAsStrings(robotSide);
         for (int i = 0; i < armJointNames.size(); i++)
         {
            privilegedConfigurationCommand.addJoint((OneDoFJointBasics) controllerSystem.findJoint(armJointNames.get(i)),
                                                    PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_MID_RANGE);
         }
      }

      registry.addChild(jointGainRegistry);
      registry.addChild(bodyGainRegistry);
      registry.addChild(momentumRegistry);
      registry.addChild(comHeightGainRegistry);

      centerOfMassFrame = new MovingCenterOfMassReferenceFrame("centerOfMass", elevator.getBodyFixedFrame(), elevator);
      controlCoreToolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                           gravityZ,
                                                           null,
                                                           controlledJoints,
                                                           centerOfMassFrame,
                                                           walkingControllerParameters.getMomentumOptimizationSettings(),
                                                           graphicsListRegistry,
                                                           registry);

      FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
      feedbackControlCommandList.addCommand(chestManager.createFeedbackControlTemplate());
      feedbackControlCommandList.addCommand(headManager.createFeedbackControlTemplate());
      for (RobotSide robotSide : RobotSide.values)
      {
         feedbackControlCommandList.addCommand(handManagers.get(robotSide).createFeedbackControlTemplate());
      }
      FeedbackControllerTemplate template = new FeedbackControllerTemplate(feedbackControlCommandList);

      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      controlCoreToolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, template, registry);
   }

   private static OneDoFJointBasics[] collectControllableJoints(FullHumanoidRobotModel fullHumanoidRobotModel, JointBasics[] jointsToIgnore)
   {
      JointBasics[] controllableJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullHumanoidRobotModel, jointsToIgnore);
      return MultiBodySystemTools.filterJoints(controllableJoints, OneDoFJointBasics.class);
   }

   private RigidBodyControlManager createRigidBodyManager(RigidBodyBasics bodyToControl,
                                                          RigidBodyBasics baseBody,
                                                          ReferenceFrame controlFrame,
                                                          ReferenceFrame baseFrame,
                                                          RigidBodyBasics elevator,
                                                          YoDouble yoTime,
                                                          YoGraphicsListRegistry graphicsListRegistry)
   {
      String bodyName = bodyToControl.getName();

      // Gains
      PID3DGainsReadOnly taskspaceOrientationGains = taskspaceOrientationGainMap.get(bodyName);
      PID3DGainsReadOnly taskspacePositionGains = taskspacePositionGainMap.get(bodyName);

      // Weights
      Vector3DReadOnly taskspaceAngularWeight = taskspaceAngularWeightMap.get(bodyName);
      Vector3DReadOnly taskspaceLinearWeight = taskspaceLinearWeightMap.get(bodyName);

      TObjectDoubleHashMap<String> homeConfiguration = walkingControllerParameters.getOrCreateJointHomeConfiguration();
      Pose3D homePose = walkingControllerParameters.getOrCreateBodyHomeConfiguration().get(bodyName);

      ContactablePlaneBody contactableBody = null;
      RigidBodyControlMode defaultControlMode = walkingControllerParameters.getDefaultControlModesForRigidBodies().get(bodyName);
      boolean enableFunctionGenerators = walkingControllerParameters.enableFunctionGeneratorMode(bodyToControl.getName());

      RigidBodyControlManager manager = new RigidBodyControlManager(bodyToControl,
                                                                    baseBody,
                                                                    elevator,
                                                                    homeConfiguration,
                                                                    homePose,
                                                                    controlFrame,
                                                                    baseFrame,
                                                                    taskspaceAngularWeight,
                                                                    taskspaceLinearWeight,
                                                                    taskspaceOrientationGains,
                                                                    taskspacePositionGains,
                                                                    contactableBody,
                                                                    null,
                                                                    defaultControlMode,
                                                                    enableFunctionGenerators,
                                                                    yoTime,
                                                                    controlDT,
                                                                    graphicsListRegistry,
                                                                    registry);
      manager.setGains(jointspaceGainMap);
      manager.setWeights(jointspaceWeightMap, userModeWeightMap);

      return manager;
   }

   @Override
   public void onEntry()
   {
      controllerCore.initialize();

      chestManager.initialize();
      headManager.initialize();

      for (RobotSide robotSide : RobotSide.values)
      {
         handManagers.get(robotSide).initialize();
      }

      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_ZERO);
   }

   @Override
   public void doAction(double timeInState)
   {
      centerOfMassFrame.update();
      for (RobotSide robotSide : RobotSide.values)
      {
         handControlFrames.get(robotSide).update();
      }

      consumePelvisCommands();
      consumeHeadCommands();
      consumeChestCommands();
      consumeGoHomeMessages();
      consumeStopAllTrajectoryCommands();
      consumeManipulationCommands();

      reportStatusMessages();

      chestManager.compute();
      headManager.compute();

      for (RobotSide robotSide : RobotSide.values)
      {
         handManagers.get(robotSide).compute();
      }

      controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);

      /* Head commands */
      controllerCoreCommand.addFeedbackControlCommand(headManager.getFeedbackControlCommand());
      controllerCoreCommand.addInverseDynamicsCommand(headManager.getInverseDynamicsCommand());

      /* Chest commands */
      controllerCoreCommand.addFeedbackControlCommand(chestManager.getFeedbackControlCommand());
      controllerCoreCommand.addInverseDynamicsCommand(chestManager.getInverseDynamicsCommand());

      /* Arm commands */
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(handManagers.get(robotSide).getFeedbackControlCommand());
         controllerCoreCommand.addInverseDynamicsCommand(handManagers.get(robotSide).getInverseDynamicsCommand());
      }

      JointDesiredOutputList stateSpecificJointSettings = getStateSpecificJointSettings();
      controllerCoreCommand.completeLowLevelJointData(stateSpecificJointSettings);

      controllerCoreTimer.startMeasurement();
      controllerCore.compute(controllerCoreCommand);
      controllerCoreTimer.stopMeasurement();
   }

   private void consumeHeadCommands()
   {
      if (commandInputManager.isNewCommandAvailable(HeadTrajectoryCommand.class))
      {
         HeadTrajectoryCommand command = commandInputManager.pollNewestCommand(HeadTrajectoryCommand.class);
         SO3TrajectoryControllerCommand so3Trajectory = command.getSO3Trajectory();
         so3Trajectory.setSequenceId(command.getSequenceId());
         headManager.handleTaskspaceTrajectoryCommand(so3Trajectory);
      }
      if (commandInputManager.isNewCommandAvailable(NeckTrajectoryCommand.class))
      {
         NeckTrajectoryCommand command = commandInputManager.pollNewestCommand(NeckTrajectoryCommand.class);
         JointspaceTrajectoryCommand jointspaceTrajectory = command.getJointspaceTrajectory();
         jointspaceTrajectory.setSequenceId(command.getSequenceId());
         headManager.handleJointspaceTrajectoryCommand(jointspaceTrajectory);
      }
      if (commandInputManager.isNewCommandAvailable(NeckDesiredAccelerationsCommand.class))
      {
         NeckDesiredAccelerationsCommand command = commandInputManager.pollNewestCommand(NeckDesiredAccelerationsCommand.class);
         DesiredAccelerationsCommand desiredAccelerations = command.getDesiredAccelerations();
         desiredAccelerations.setSequenceId(command.getSequenceId());
         headManager.handleDesiredAccelerationsCommand(desiredAccelerations);
      }
      if (commandInputManager.isNewCommandAvailable(HeadHybridJointspaceTaskspaceTrajectoryCommand.class))
      {
         HeadHybridJointspaceTaskspaceTrajectoryCommand command = commandInputManager.pollNewestCommand(HeadHybridJointspaceTaskspaceTrajectoryCommand.class);
         SO3TrajectoryControllerCommand taskspaceTrajectoryCommand = command.getTaskspaceTrajectoryCommand();
         JointspaceTrajectoryCommand jointspaceTrajectoryCommand = command.getJointspaceTrajectoryCommand();
         taskspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         jointspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         headManager.handleHybridTrajectoryCommand(taskspaceTrajectoryCommand, jointspaceTrajectoryCommand);
      }
   }

   private void consumeChestCommands()
   {
      if (commandInputManager.isNewCommandAvailable(ChestTrajectoryCommand.class))
      {
         ChestTrajectoryCommand command = commandInputManager.pollNewestCommand(ChestTrajectoryCommand.class);
         SO3TrajectoryControllerCommand so3Trajectory = command.getSO3Trajectory();
         so3Trajectory.setSequenceId(command.getSequenceId());
         so3Trajectory.setTrajectoryFrame(ReferenceFrame.getWorldFrame());
         chestManager.handleTaskspaceTrajectoryCommand(so3Trajectory);
      }
      if (commandInputManager.isNewCommandAvailable(SpineTrajectoryCommand.class))
      {
         SpineTrajectoryCommand command = commandInputManager.pollNewestCommand(SpineTrajectoryCommand.class);
         JointspaceTrajectoryCommand jointspaceTrajectory = command.getJointspaceTrajectory();
         jointspaceTrajectory.setSequenceId(command.getSequenceId());
         chestManager.handleJointspaceTrajectoryCommand(jointspaceTrajectory);
      }
      if (commandInputManager.isNewCommandAvailable(SpineDesiredAccelerationsCommand.class))
      {
         SpineDesiredAccelerationsCommand command = commandInputManager.pollNewestCommand(SpineDesiredAccelerationsCommand.class);
         DesiredAccelerationsCommand desiredAccelerations = command.getDesiredAccelerations();
         desiredAccelerations.setSequenceId(command.getSequenceId());
         chestManager.handleDesiredAccelerationsCommand(desiredAccelerations);
      }
      if (commandInputManager.isNewCommandAvailable(ChestHybridJointspaceTaskspaceTrajectoryCommand.class))
      {
         ChestHybridJointspaceTaskspaceTrajectoryCommand command = commandInputManager.pollNewestCommand(ChestHybridJointspaceTaskspaceTrajectoryCommand.class);
         SO3TrajectoryControllerCommand taskspaceTrajectoryCommand = command.getTaskspaceTrajectoryCommand();
         JointspaceTrajectoryCommand jointspaceTrajectoryCommand = command.getJointspaceTrajectoryCommand();
         taskspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         jointspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
         chestManager.handleHybridTrajectoryCommand(taskspaceTrajectoryCommand, jointspaceTrajectoryCommand);
      }
   }

   private void consumePelvisCommands()
   {
      // TODO (AM): Alex is fixed base for now, no pelvis commands
      commandInputManager.clearCommands(PelvisTrajectoryCommand.class);
   }

   private void consumeGoHomeMessages()
   {
      if (!commandInputManager.isNewCommandAvailable(GoHomeCommand.class))
         return;

      List<GoHomeCommand> commands = commandInputManager.pollNewCommands(GoHomeCommand.class);
      for (int i = 0; i < commands.size(); i++)
      {
         GoHomeCommand command = commands.get(i);

         for (RobotSide robotSide : RobotSide.values)
         {
            if (command.getRequest(robotSide, HumanoidBodyPart.ARM))
            {
               RigidBodyControlManager handManager = handManagers.get(robotSide);
               if (handManager != null)
               {
                  handManager.goHome(command.getTrajectoryTime());
               }
            }
         }

         if (command.getRequest(HumanoidBodyPart.CHEST))
         {
            chestManager.goHome(command.getTrajectoryTime());
         }
      }
   }

   private void consumeStopAllTrajectoryCommands()
   {
      if (!commandInputManager.isNewCommandAvailable(StopAllTrajectoryCommand.class))
         return;

      StopAllTrajectoryCommand command = commandInputManager.pollNewestCommand(StopAllTrajectoryCommand.class);
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).handleStopAllTrajectoryCommand(command);
      }

      if (chestManager != null)
      {
         chestManager.handleStopAllTrajectoryCommand(command);
      }
   }

   private void consumeManipulationCommands()
   {
      List<HandTrajectoryCommand> handTrajectoryCommands = commandInputManager.pollNewCommands(HandTrajectoryCommand.class);
      List<HandWrenchTrajectoryCommand> handWrenchTrajectoryCommands = commandInputManager.pollNewCommands(HandWrenchTrajectoryCommand.class);
      List<ArmTrajectoryCommand> armTrajectoryCommands = commandInputManager.pollNewCommands(ArmTrajectoryCommand.class);
      List<ArmDesiredAccelerationsCommand> armDesiredAccelerationCommands = commandInputManager.pollNewCommands(ArmDesiredAccelerationsCommand.class);
      List<HandHybridJointspaceTaskspaceTrajectoryCommand> handHybridCommands = commandInputManager.pollNewCommands(
            HandHybridJointspaceTaskspaceTrajectoryCommand.class);

      for (int i = 0; i < handTrajectoryCommands.size(); i++)
      {
         HandTrajectoryCommand command = handTrajectoryCommands.get(i);
         RobotSide robotSide = command.getRobotSide();
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         if (handManager != null)
         {
            SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
            se3Trajectory.setSequenceId(command.getSequenceId());
            handManager.handleTaskspaceTrajectoryCommand(se3Trajectory);
         }
      }

      for (int i = 0; i < handWrenchTrajectoryCommands.size(); i++)
      {
         HandWrenchTrajectoryCommand command = handWrenchTrajectoryCommands.get(i);
         RobotSide robotSide = command.getRobotSide();
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         if (handManager != null)
         {
            WrenchTrajectoryControllerCommand wrenchTrajectory = command.getWrenchTrajectory();
            wrenchTrajectory.setSequenceId(command.getSequenceId());
            handManager.handleWrenchTrajectoryCommand(wrenchTrajectory);
         }
      }
      for (int i = 0; i < armTrajectoryCommands.size(); i++)
      {
         ArmTrajectoryCommand command = armTrajectoryCommands.get(i);
         RobotSide robotSide = command.getRobotSide();
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         if (handManager != null)
         {
            JointspaceTrajectoryCommand jointspaceTrajectory = command.getJointspaceTrajectory();
            jointspaceTrajectory.setSequenceId(command.getSequenceId());
            handManager.handleJointspaceTrajectoryCommand(jointspaceTrajectory);
         }
      }

      for (int i = 0; i < handHybridCommands.size(); i++)
      {
         HandHybridJointspaceTaskspaceTrajectoryCommand command = handHybridCommands.get(i);
         RobotSide robotSide = command.getRobotSide();
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         if (handManager != null)
         {
            SE3TrajectoryControllerCommand taskspaceTrajectoryCommand = command.getTaskspaceTrajectoryCommand();
            JointspaceTrajectoryCommand jointspaceTrajectoryCommand = command.getJointspaceTrajectoryCommand();
            taskspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
            jointspaceTrajectoryCommand.setSequenceId(command.getSequenceId());
            handManager.handleHybridTrajectoryCommand(taskspaceTrajectoryCommand, jointspaceTrajectoryCommand);
         }
      }

      for (int i = 0; i < armDesiredAccelerationCommands.size(); i++)
      {
         ArmDesiredAccelerationsCommand command = armDesiredAccelerationCommands.get(i);
         RobotSide robotSide = command.getRobotSide();
         RigidBodyControlManager handManager = handManagers.get(robotSide);
         if (handManager != null)
         {
            DesiredAccelerationsCommand desiredAccelerations = command.getDesiredAccelerations();
            desiredAccelerations.setSequenceId(command.getSequenceId());
            handManager.handleDesiredAccelerationsCommand(desiredAccelerations);
         }
      }

   }

   private void reportStatusMessages()
   {
      Object statusMessage;

      for (RobotSide robotSide : RobotSide.values)
      {
         statusMessage = handManagers.get(robotSide).pollStatusToReport();
         if (statusMessage != null)
            statusMessageOutputManager.reportStatusMessage(statusMessage);
      }

      if (chestManager != null)
      {
         statusMessage = chestManager.pollStatusToReport();
         if (statusMessage != null)
            statusMessageOutputManager.reportStatusMessage(statusMessage);
      }

      if (headManager != null)
      {
         statusMessage = headManager.pollStatusToReport();
         if (statusMessage != null)
            statusMessageOutputManager.reportStatusMessage(statusMessage);
      }
  }


   @Override
   public void onExit(double timeInState)
   {

   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }
}
