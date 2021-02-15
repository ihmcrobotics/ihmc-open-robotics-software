package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDSE3Gains;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

public class JumpingControlManagerFactory
{
   public static final String weightRegistryName = "MomentumOptimizationSettings";
   public static final String jointspaceGainRegistryName = "JointspaceGains";
   public static final String rigidBodyGainRegistryName = "RigidBodyGains";
   public static final String footGainRegistryName = "FootGains";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoRegistry momentumRegistry = new YoRegistry(weightRegistryName);
   private final YoRegistry jointGainRegistry = new YoRegistry(jointspaceGainRegistryName);
   private final YoRegistry bodyGainRegistry = new YoRegistry(rigidBodyGainRegistryName);
   private final YoRegistry footGainRegistry = new YoRegistry(footGainRegistryName);

   private JumpingFeetManager feetManager;
   private JumpingPelvisOrientationManager pelvisOrientationManager;
   private JumpingBalanceManager balanceManager;

   private final Map<String, RigidBodyControlManager> rigidBodyManagerMapByBodyName = new HashMap<>();

   private JumpingControllerToolbox controllerToolbox;
   private WalkingControllerParameters walkingControllerParameters;
   private CoPTrajectoryParameters copTrajectoryParameters;
   private JumpingCoPTrajectoryParameters jumpingCopTrajectoryParameters;
   private JumpingParameters jumpingParameters;
   private MomentumOptimizationSettings momentumOptimizationSettings;

   private final Map<String, PIDGainsReadOnly> jointGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspaceOrientationGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspacePositionGainMap = new HashMap<>();

   private final Map<String, DoubleProvider> jointspaceWeightMap = new HashMap<>();
   private final Map<String, DoubleProvider> userModeWeightMap = new HashMap<>();
   private final Map<String, Vector3DReadOnly> taskspaceAngularWeightMap = new HashMap<>();
   private final Map<String, Vector3DReadOnly> taskspaceLinearWeightMap = new HashMap<>();
   private Vector3DReadOnly loadedFootAngularWeight;
   private Vector3DReadOnly loadedFootLinearWeight;
   private PIDSE3GainsReadOnly swingFootGains;

   public JumpingControlManagerFactory(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      parentRegistry.addChild(momentumRegistry);
      parentRegistry.addChild(jointGainRegistry);
      parentRegistry.addChild(bodyGainRegistry);
      parentRegistry.addChild(footGainRegistry);
   }

   public void setHighLevelHumanoidControllerToolbox(JumpingControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
   }

   public void setCoPTrajectoryParameters(CoPTrajectoryParameters copTrajectoryParameters)
   {
      this.copTrajectoryParameters = copTrajectoryParameters;
   }

   public void setJumpingCoPTrajectoryParameters(JumpingCoPTrajectoryParameters jumpingCopTrajectoryParameters)
   {
      this.jumpingCopTrajectoryParameters = jumpingCopTrajectoryParameters;
   }

   public void setJumpingParameters(JumpingParameters jumpingCopTrajectoryParameters)
   {
      this.jumpingParameters = jumpingCopTrajectoryParameters;
   }

   public void setWalkingControllerParameters(WalkingControllerParameters walkingControllerParameters)
   {
      this.walkingControllerParameters = walkingControllerParameters;
      momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();

      // Transform weights and gains to their parameterized versions.
      ParameterTools.extractJointGainMap(walkingControllerParameters.getJointSpaceControlGains(), jointGainMap, jointGainRegistry);
      ParameterTools.extract3DGainMap("Orientation", walkingControllerParameters.getTaskspaceOrientationControlGains(), taskspaceOrientationGainMap, bodyGainRegistry);
      ParameterTools.extract3DGainMap("Position", walkingControllerParameters.getTaskspacePositionControlGains(), taskspacePositionGainMap, bodyGainRegistry);
      ParameterTools.extractJointWeightMap("JointspaceWeight", momentumOptimizationSettings.getJointspaceWeights(), jointspaceWeightMap, momentumRegistry);
      ParameterTools.extractJointWeightMap("UserModeWeight", momentumOptimizationSettings.getUserModeWeights(), userModeWeightMap, momentumRegistry);
      ParameterTools.extract3DWeightMap("AngularWeight", momentumOptimizationSettings.getTaskspaceAngularWeights(), taskspaceAngularWeightMap, momentumRegistry);
      ParameterTools.extract3DWeightMap("LinearWeight", momentumOptimizationSettings.getTaskspaceLinearWeights(), taskspaceLinearWeightMap, momentumRegistry);

      loadedFootAngularWeight = new ParameterVector3D("LoadedFootAngularWeight", momentumOptimizationSettings.getLoadedFootAngularWeight(), momentumRegistry);
      loadedFootLinearWeight = new ParameterVector3D("LoadedFootLinearWeight", momentumOptimizationSettings.getLoadedFootLinearWeight(), momentumRegistry);

      swingFootGains = new ParameterizedPIDSE3Gains("SwingFoot", walkingControllerParameters.getSwingFootControlGains(), footGainRegistry);
   }

   public RigidBodyControlManager getOrCreateRigidBodyManager(RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, ReferenceFrame controlFrame,
                                                              ReferenceFrame baseFrame)
   {
      if (bodyToControl == null)
         return null;

      String bodyName = bodyToControl.getName();
      if (rigidBodyManagerMapByBodyName.containsKey(bodyName))
      {
         RigidBodyControlManager manager = rigidBodyManagerMapByBodyName.get(bodyName);
         if (manager != null)
            return manager;
      }

      if (!hasWalkingControllerParameters(RigidBodyControlManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(RigidBodyControlManager.class))
         return null;
      if (!hasHighLevelHumanoidControllerToolbox(BalanceManager.class))
         return null;

      // Gains
      PID3DGainsReadOnly taskspaceOrientationGains = taskspaceOrientationGainMap.get(bodyName);
      PID3DGainsReadOnly taskspacePositionGains = taskspacePositionGainMap.get(bodyName);

      // Weights
      Vector3DReadOnly taskspaceAngularWeight = taskspaceAngularWeightMap.get(bodyName);
      Vector3DReadOnly taskspaceLinearWeight = taskspaceLinearWeightMap.get(bodyName);

      TObjectDoubleHashMap<String> homeConfiguration = walkingControllerParameters.getOrCreateJointHomeConfiguration();
      Pose3D homePose = walkingControllerParameters.getOrCreateBodyHomeConfiguration().get(bodyName);
      RigidBodyBasics elevator = controllerToolbox.getFullRobotModel().getElevator();
      YoDouble yoTime = controllerToolbox.getYoTime();

      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      RigidBodyControlMode defaultControlMode = walkingControllerParameters.getDefaultControlModesForRigidBodies().get(bodyName);

      RigidBodyControlManager manager = new RigidBodyControlManager(bodyToControl, baseBody, elevator, homeConfiguration, homePose, controlFrame, baseFrame,
                                                                    taskspaceAngularWeight, taskspaceLinearWeight, taskspaceOrientationGains,
                                                                    taskspacePositionGains, null, defaultControlMode, yoTime, graphicsListRegistry, registry);
      manager.setGains(jointGainMap);
      manager.setWeights(jointspaceWeightMap, userModeWeightMap);

      rigidBodyManagerMapByBodyName.put(bodyName, manager);
      return manager;
   }

   public JumpingFeetManager getOrCreateFeetManager()
   {
      if (feetManager != null)
         return feetManager;

      if (!hasHighLevelHumanoidControllerToolbox(JumpingFeetManager.class))
         return null;
      if (!hasWalkingControllerParameters(JumpingFeetManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(JumpingFeetManager.class))
         return null;

      feetManager = new JumpingFeetManager(controllerToolbox,
                                    walkingControllerParameters,
                                    swingFootGains,
                                    registry);

      String footName = controllerToolbox.getFullRobotModel().getFoot(RobotSide.LEFT).getName();
      Vector3DReadOnly angularWeight = taskspaceAngularWeightMap.get(footName);
      Vector3DReadOnly linearWeight = taskspaceLinearWeightMap.get(footName);
      if (angularWeight == null || linearWeight == null)
      {
         throw new RuntimeException("Not all weights defined for the foot control: " + footName + " needs weights.");
      }
      String otherFootName = controllerToolbox.getFullRobotModel().getFoot(RobotSide.RIGHT).getName();
      if (taskspaceAngularWeightMap.get(otherFootName) != angularWeight || taskspaceLinearWeightMap.get(otherFootName) != linearWeight)
      {
         throw new RuntimeException("There can only be one weight defined for both feet. Make sure they are in the same GroupParameter");
      }
      feetManager.setWeights(loadedFootAngularWeight, loadedFootLinearWeight, angularWeight, linearWeight);

      return feetManager;
   }

   public JumpingBalanceManager getOrCreateBalanceManager()
   {
      if (balanceManager != null)
         return balanceManager;

      if (!hasHighLevelHumanoidControllerToolbox(JumpingBalanceManager.class))
         return null;
      if (!hasCoPTrajectoryParameters(JumpingBalanceManager.class))
         return null;
      if (!hasJumpingCoPTrajectoryParameters(JumpingBalanceManager.class))
         return null;

      balanceManager = new JumpingBalanceManager(controllerToolbox, copTrajectoryParameters, jumpingCopTrajectoryParameters, jumpingParameters, registry);

      return balanceManager;
   }

   public JumpingPelvisOrientationManager getOrCreatePelvisOrientationManager()
   {
      if (pelvisOrientationManager != null)
         return pelvisOrientationManager;

      if (!hasHighLevelHumanoidControllerToolbox(JumpingPelvisOrientationManager.class))
         return null;
      if (!hasWalkingControllerParameters(JumpingPelvisOrientationManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(JumpingPelvisOrientationManager.class))
         return null;

      String pelvisName = controllerToolbox.getFullRobotModel().getPelvis().getName();
      PID3DGainsReadOnly pelvisGains = taskspaceOrientationGainMap.get(pelvisName);
      Vector3DReadOnly pelvisAngularWeight = taskspaceAngularWeightMap.get(pelvisName);

      pelvisOrientationManager = new JumpingPelvisOrientationManager(pelvisGains, controllerToolbox,
                                                              registry);
      pelvisOrientationManager.setWeights(pelvisAngularWeight);
      pelvisOrientationManager.setPrepareForLocomotion(walkingControllerParameters.doPreparePelvisForLocomotion());
      return pelvisOrientationManager;
   }

   private boolean hasHighLevelHumanoidControllerToolbox(Class<?> managerClass)
   {
      if (controllerToolbox != null)
         return true;
      missingObjectWarning(JumpingControllerToolbox.class, managerClass);
      return false;
   }

   private boolean hasWalkingControllerParameters(Class<?> managerClass)
   {
      if (walkingControllerParameters != null)
         return true;
      missingObjectWarning(WalkingControllerParameters.class, managerClass);
      return false;
   }

   private boolean hasCoPTrajectoryParameters(Class<?> managerClass)
   {
      if (copTrajectoryParameters != null)
         return true;
      missingObjectWarning(CoPTrajectoryParameters.class, managerClass);
      return false;
   }

   private boolean hasJumpingCoPTrajectoryParameters(Class<?> managerClass)
   {
      if (jumpingCopTrajectoryParameters != null)
         return true;
      missingObjectWarning(JumpingCoPTrajectoryParameters.class, managerClass);
      return false;
   }

   private boolean hasMomentumOptimizationSettings(Class<?> managerClass)
   {
      if (momentumOptimizationSettings != null)
         return true;
      missingObjectWarning(MomentumOptimizationSettings.class, managerClass);
      return false;
   }

   private void missingObjectWarning(Class<?> missingObjectClass, Class<?> managerClass)
   {
      LogTools.warn(missingObjectClass.getSimpleName() + " has not been set, cannot create: " + managerClass.getSimpleName());
   }

   public void initializeManagers()
   {
      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initialize();

      Collection<RigidBodyControlManager> bodyManagers = rigidBodyManagerMapByBodyName.values();
      for (RigidBodyControlManager bodyManager : bodyManagers)
      {
         if (bodyManager != null)
            bodyManager.initialize();
      }
   }

   public FeedbackControllerTemplate createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();

      if (feetManager != null)
      {
         FeedbackControlCommandList template = feetManager.createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }

      Collection<RigidBodyControlManager> bodyManagers = rigidBodyManagerMapByBodyName.values();
      for (RigidBodyControlManager bodyManager : bodyManagers)
      {
         if (bodyManager != null)
            ret.addCommand(bodyManager.createFeedbackControlTemplate());
      }

      if (pelvisOrientationManager != null)
      {
         ret.addCommand(pelvisOrientationManager.createFeedbackControlTemplate());
      }

      return new FeedbackControllerTemplate(ret);
   }
}
