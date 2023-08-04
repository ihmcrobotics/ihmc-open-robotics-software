package us.ihmc.valkyrieRosControl.upperBody;

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
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;
import us.ihmc.mecano.frames.MovingCenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ValkyrieUpperBodyManipulationState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerName = HighLevelControllerName.WALKING;

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

   private final Map<String, PIDGainsReadOnly> jointspaceHighLevelGainMap = new HashMap<>();
   private final Map<String, PIDGainsReadOnly> jointspaceLowLevelGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspaceOrientationGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspacePositionGainMap = new HashMap<>();
   private final Map<String, DoubleProvider> jointspaceWeightMap = new HashMap<>();
   private final Map<String, DoubleProvider> userModeWeightMap = new HashMap<>();

   private final CommandInputManager commandInputManager;
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

   public ValkyrieUpperBodyManipulationState(CommandInputManager commandInputManager,
                                             double controlDT,
                                             HumanoidJointNameMap jointNameMap,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters,
                                             RigidBodyBasics rootBody,
                                             OneDoFJointBasics[] controlledJoints,
                                             YoDouble yoTime,
                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      super(controllerName, highLevelControllerParameters, controlledJoints);

      this.commandInputManager = commandInputManager;
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      ParameterTools.extractJointGainMap(walkingControllerParameters.getHighLevelJointSpaceControlGains(), jointspaceHighLevelGainMap, jointGainRegistry);
      ParameterTools.extractJointGainMap(walkingControllerParameters.getLowLevelJointSpaceControlGains(), jointspaceLowLevelGainMap, jointGainRegistry);
      ParameterTools.extract3DGainMap("Orientation", walkingControllerParameters.getTaskspaceOrientationControlGains(), taskspaceOrientationGainMap, bodyGainRegistry);
      ParameterTools.extract3DGainMap("Position", walkingControllerParameters.getTaskspacePositionControlGains(), taskspacePositionGainMap, bodyGainRegistry);
      ParameterTools.extractJointWeightMap("JointspaceWeight", momentumOptimizationSettings.getJointspaceWeights(), jointspaceWeightMap, momentumRegistry);
      ParameterTools.extractJointWeightMap("UserModeWeight", momentumOptimizationSettings.getUserModeWeights(), userModeWeightMap, momentumRegistry);
      ParameterTools.extract3DWeightMap("AngularWeight", momentumOptimizationSettings.getTaskspaceAngularWeights(), taskspaceAngularWeightMap, momentumRegistry);
      ParameterTools.extract3DWeightMap("LinearWeight", momentumOptimizationSettings.getTaskspaceLinearWeights(), taskspaceLinearWeightMap, momentumRegistry);

      String chestName = jointNameMap.getChestName();
      String pelvisName = jointNameMap.getPelvisName();
      String headName = jointNameMap.getHeadName();

      controllerSystem = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody);

      RigidBodyBasics elevator = controllerSystem.getRootBody();
      RigidBodyBasics chest = controllerSystem.findRigidBody(chestName);
      RigidBodyBasics head = controllerSystem.findRigidBody(headName);

      chestManager = createRigidBodyManager(chest, elevator, chest.getBodyFixedFrame(), elevator.getBodyFixedFrame(), yoTime, elevator, graphicsListRegistry);
      headManager = createRigidBodyManager(head, chest, head.getBodyFixedFrame(), chest.getBodyFixedFrame(), yoTime, elevator, graphicsListRegistry);

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

         RigidBodyControlManager handManager = createRigidBodyManager(hand, chest, handControlFrames.get(robotSide), chest.getBodyFixedFrame(), yoTime, elevator, graphicsListRegistry);
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
                                                           ValkyrieRosControlController.gravity,
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

   private RigidBodyControlManager createRigidBodyManager(RigidBodyBasics bodyToControl,
                                                          RigidBodyBasics baseBody,
                                                          ReferenceFrame controlFrame,
                                                          ReferenceFrame baseFrame,
                                                          YoDouble yoTime,
                                                          RigidBodyBasics elevator,
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
                                                                    defaultControlMode,
                                                                    yoTime,
                                                                    graphicsListRegistry,
                                                                    registry);
      manager.setGains(jointspaceHighLevelGainMap, jointspaceLowLevelGainMap);
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

      controllerCoreTimer.startMeasurement();
      controllerCore.compute(controllerCoreCommand);
      controllerCoreTimer.stopMeasurement();
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
