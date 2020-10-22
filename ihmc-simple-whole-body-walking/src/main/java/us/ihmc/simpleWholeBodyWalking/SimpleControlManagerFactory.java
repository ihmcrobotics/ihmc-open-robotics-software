package us.ihmc.simpleWholeBodyWalking;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
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

public class SimpleControlManagerFactory
{
   public static final String weightRegistryName = "MomentumOptimizationSettings";
   public static final String jointspaceGainRegistryName = "JointspaceGains";
   public static final String rigidBodyGainRegistryName = "RigidBodyGains";
   public static final String footGainRegistryName = "FootGains";
   public static final String comHeightGainRegistryName = "ComHeightGains";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoRegistry momentumRegistry = new YoRegistry(weightRegistryName);
   private final YoRegistry jointGainRegistry = new YoRegistry(jointspaceGainRegistryName);
   private final YoRegistry bodyGainRegistry = new YoRegistry(rigidBodyGainRegistryName);
   private final YoRegistry footGainRegistry = new YoRegistry(footGainRegistryName);
   private final YoRegistry comHeightGainRegistry = new YoRegistry(comHeightGainRegistryName);

   private SimpleBalanceManager balanceManager;
   private SimpleCenterOfMassHeightManager centerOfMassHeightManager;
   private SimpleFeetManager feetManager;
   private SimplePelvisOrientationManager pelvisOrientationManager;

   private final Map<String, RigidBodyControlManager> rigidBodyManagerMapByBodyName = new HashMap<>();

   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private WalkingControllerParameters walkingControllerParameters;
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
   private PIDSE3GainsReadOnly holdFootGains;

   private PIDGainsReadOnly walkingControllerComHeightGains;
   private DoubleProvider walkingControllerMaxComHeightVelocity;

   public SimpleControlManagerFactory(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      parentRegistry.addChild(momentumRegistry);
      parentRegistry.addChild(jointGainRegistry);
      parentRegistry.addChild(bodyGainRegistry);
      parentRegistry.addChild(footGainRegistry);
      parentRegistry.addChild(comHeightGainRegistry);
   }

   public void setHighLevelHumanoidControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
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
      holdFootGains = new ParameterizedPIDSE3Gains("HoldFoot", walkingControllerParameters.getHoldPositionFootControlGains(), footGainRegistry);

      walkingControllerComHeightGains = new ParameterizedPIDGains("WalkingControllerComHeight", walkingControllerParameters.getCoMHeightControlGains(), comHeightGainRegistry);
      walkingControllerMaxComHeightVelocity = new DoubleParameter("MaximumVelocityWalkingControllerComHeight", comHeightGainRegistry, walkingControllerParameters.getMaximumVelocityCoMHeight());
   }

   public SimpleBalanceManager getOrCreateBalanceManager()
   {
      if (balanceManager != null)
         return balanceManager;

      if (!hasHighLevelHumanoidControllerToolbox(BalanceManager.class))
         return null;
      if (!hasWalkingControllerParameters(BalanceManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(BalanceManager.class))
         return null;

      balanceManager = new SimpleBalanceManager(controllerToolbox, walkingControllerParameters, registry);
      return balanceManager;
   }

   public SimpleCenterOfMassHeightManager getOrCreateCenterOfMassHeightManager()
   {
      if (centerOfMassHeightManager != null)
         return centerOfMassHeightManager;

      if (!hasHighLevelHumanoidControllerToolbox(CenterOfMassHeightManager.class))
         return null;
      if (!hasWalkingControllerParameters(CenterOfMassHeightManager.class))
         return null;

      centerOfMassHeightManager = new SimpleCenterOfMassHeightManager(controllerToolbox, walkingControllerParameters, registry);
      centerOfMassHeightManager.setComHeightGains(walkingControllerComHeightGains, walkingControllerMaxComHeightVelocity);
      return centerOfMassHeightManager;
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

      ContactablePlaneBody contactableBody = controllerToolbox.getContactableBody(bodyToControl);
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      RigidBodyControlMode defaultControlMode = walkingControllerParameters.getDefaultControlModesForRigidBodies().get(bodyName);

      RigidBodyControlManager manager = new RigidBodyControlManager(bodyToControl, baseBody, elevator, homeConfiguration, homePose, controlFrame, baseFrame,
                                                                    taskspaceAngularWeight, taskspaceLinearWeight, taskspaceOrientationGains,
                                                                    taskspacePositionGains, contactableBody, defaultControlMode, yoTime, graphicsListRegistry, registry);
      manager.setGains(jointGainMap);
      manager.setWeights(jointspaceWeightMap, userModeWeightMap);

      rigidBodyManagerMapByBodyName.put(bodyName, manager);
      return manager;
   }

   public SimpleFeetManager getOrCreateFeetManager()
   {
      if (feetManager != null)
         return feetManager;

      if (!hasHighLevelHumanoidControllerToolbox(FeetManager.class))
         return null;
      if (!hasWalkingControllerParameters(FeetManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(FeetManager.class))
         return null;

      feetManager = new SimpleFeetManager(controllerToolbox, walkingControllerParameters, swingFootGains, holdFootGains, registry);

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

   public SimplePelvisOrientationManager getOrCreatePelvisOrientationManager()
   {
      if (pelvisOrientationManager != null)
         return pelvisOrientationManager;

      if (!hasHighLevelHumanoidControllerToolbox(PelvisOrientationManager.class))
         return null;
      if (!hasWalkingControllerParameters(PelvisOrientationManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(PelvisOrientationManager.class))
         return null;

      String pelvisName = controllerToolbox.getFullRobotModel().getPelvis().getName();
      PID3DGainsReadOnly pelvisGains = taskspaceOrientationGainMap.get(pelvisName);
      Vector3DReadOnly pelvisAngularWeight = taskspaceAngularWeightMap.get(pelvisName);

      pelvisOrientationManager = new SimplePelvisOrientationManager(pelvisGains, controllerToolbox, registry);
      pelvisOrientationManager.setWeights(pelvisAngularWeight);
      pelvisOrientationManager.setPrepareForLocomotion(walkingControllerParameters.doPreparePelvisForLocomotion());
      return pelvisOrientationManager;
   }

   private boolean hasHighLevelHumanoidControllerToolbox(Class<?> managerClass)
   {
      if (controllerToolbox != null)
         return true;
      missingObjectWarning(HighLevelHumanoidControllerToolbox.class, managerClass);
      return false;
   }

   private boolean hasWalkingControllerParameters(Class<?> managerClass)
   {
      if (walkingControllerParameters != null)
         return true;
      missingObjectWarning(WalkingControllerParameters.class, managerClass);
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
      if (balanceManager != null)
         balanceManager.initialize();
      if (centerOfMassHeightManager != null)
         centerOfMassHeightManager.initialize();
      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initialize();

      Collection<RigidBodyControlManager> bodyManagers = rigidBodyManagerMapByBodyName.values();
      for (RigidBodyControlManager bodyManager : bodyManagers)
      {
         if (bodyManager != null)
            bodyManager.initialize();
      }
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
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

      if (centerOfMassHeightManager != null)
         ret.addCommand(centerOfMassHeightManager.createFeedbackControlTemplate());

      if (pelvisOrientationManager != null)
      {
         ret.addCommand(pelvisOrientationManager.createFeedbackControlTemplate());
      }

      return ret;
   }
}
