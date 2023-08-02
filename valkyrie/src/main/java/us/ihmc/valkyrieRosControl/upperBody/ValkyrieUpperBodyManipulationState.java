package us.ihmc.valkyrieRosControl.upperBody;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.Map;

public class ValkyrieUpperBodyManipulationState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerName = HighLevelControllerName.WALKING;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

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

   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final Map<String, PIDGainsReadOnly> jointspaceHighLevelGainMap = new HashMap<>();
   private final Map<String, PIDGainsReadOnly> jointspaceLowLevelGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspaceOrientationGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspacePositionGainMap = new HashMap<>();
   private final Map<String, DoubleProvider> jointspaceWeightMap = new HashMap<>();
   private final Map<String, DoubleProvider> userModeWeightMap = new HashMap<>();

   private final HighLevelControllerParameters highLevelControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final RigidBodyControlManager chestManager;
   private final RigidBodyControlManager headManager;
   private final MovingZUpFrame pelvisZUpFrame;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   public ValkyrieUpperBodyManipulationState(HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters,
                                             FullHumanoidRobotModel fullRobotModel,
                                             OneDoFJointBasics[] controlledJoints,
                                             YoDouble yoTime,
                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      super(controllerName, highLevelControllerParameters, controlledJoints);

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

      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics head = fullRobotModel.getHead();

      MovingReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
      ReferenceFrame modelStationaryFrame = fullRobotModel.getModelStationaryFrame();
      pelvisZUpFrame = new MovingZUpFrame(pelvisFrame, modelStationaryFrame, "pelvisZUpFrame");

      chestManager = createRigidBodyManager(chest, pelvis, chest.getBodyFixedFrame(), pelvisZUpFrame, yoTime, fullRobotModel.getElevator(), graphicsListRegistry);
      headManager = createRigidBodyManager(head, chest, head.getBodyFixedFrame(), chest.getBodyFixedFrame(), yoTime, fullRobotModel.getElevator(), graphicsListRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         RigidBodyControlManager handManager = createRigidBodyManager(hand, chest, handControlFrame, chest.getBodyFixedFrame(), yoTime, fullRobotModel.getElevator(), graphicsListRegistry);
         handManager.setDoPrepareForLocomotion(false);
         handManagers.put(robotSide, handManager);
      }

      registry.addChild(jointGainRegistry);
      registry.addChild(bodyGainRegistry);
      registry.addChild(momentumRegistry);
      registry.addChild(comHeightGainRegistry);
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
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public void onEntry()
   {
      chestManager.initialize();
      headManager.initialize();

      for (RobotSide robotSide : RobotSide.values)
      {
         handManagers.get(robotSide).initialize();
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      chestManager.compute();
      headManager.compute();

      for (RobotSide robotSide : RobotSide.values)
      {
         handManagers.get(robotSide).compute();
      }
   }

   @Override
   public void onExit(double timeInState)
   {

   }
}
