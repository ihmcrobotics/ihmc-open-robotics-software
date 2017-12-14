package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HighLevelControlManagerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StatusMessageOutputManager statusOutputManager;

   private BalanceManager balanceManager;
   private CenterOfMassHeightManager centerOfMassHeightManager;
   private FeetManager feetManager;
   private PelvisOrientationManager pelvisOrientationManager;
   private LegConfigurationManager legConfigurationManager;

   private final Map<String, RigidBodyControlManager> rigidBodyManagerMapByBodyName = new HashMap<>();

   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private WalkingControllerParameters walkingControllerParameters;
   private ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   private ICPAngularMomentumModifierParameters angularMomentumModifierParameters;
   private MomentumOptimizationSettings momentumOptimizationSettings;

   private final Map<String, PIDGainsReadOnly> jointGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspaceOrientationGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspacePositionGainMap = new HashMap<>();

   private final Map<String, DoubleProvider> jointspaceWeightMap = new HashMap<>();
   private final Map<String, DoubleProvider> userModeWeightMap = new HashMap<>();
   private final Map<String, Vector3DReadOnly> taskspaceAngularWeightMap = new HashMap<>();
   private final Map<String, Vector3DReadOnly> taskspaceLinearWeightMap = new HashMap<>();

   public HighLevelControlManagerFactory(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;
      parentRegistry.addChild(registry);
   }

   public void setHighLevelHumanoidControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
   }

   public void setWalkingControllerParameters(WalkingControllerParameters walkingControllerParameters)
   {
      this.walkingControllerParameters = walkingControllerParameters;
      momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      angularMomentumModifierParameters = walkingControllerParameters.getICPAngularMomentumModifierParameters();

      // Transform weights and gains to their parametrized versions.
      ParameterTools.extractJointGainMap(walkingControllerParameters.getJointSpaceControlGains(), jointGainMap, registry);
      ParameterTools.extract3DGainMap("Orientation", walkingControllerParameters.getTaskspaceOrientationControlGains(), taskspaceOrientationGainMap, registry);
      ParameterTools.extract3DGainMap("Position", walkingControllerParameters.getTaskspacePositionControlGains(), taskspacePositionGainMap, registry);
      ParameterTools.extractJointWeightMap("JointspaceWeight", momentumOptimizationSettings.getJointspaceWeights(), jointspaceWeightMap, registry);
      ParameterTools.extractJointWeightMap("UserModeWeight", momentumOptimizationSettings.getUserModeWeights(), userModeWeightMap, registry);
      ParameterTools.extract3DWeightMap("AngularWeight", momentumOptimizationSettings.getTaskspaceAngularWeights(), taskspaceAngularWeightMap, registry);
      ParameterTools.extract3DWeightMap("LinearWeight", momentumOptimizationSettings.getTaskspaceLinearWeights(), taskspaceLinearWeightMap, registry);
   }

   public void setCapturePointPlannerParameters(ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters)
   {
      this.capturePointPlannerParameters = capturePointPlannerParameters;
   }

   public BalanceManager getOrCreateBalanceManager()
   {
      if (balanceManager != null)
         return balanceManager;

      if (!hasHighLevelHumanoidControllerToolbox(BalanceManager.class))
         return null;
      if (!hasWalkingControllerParameters(BalanceManager.class))
         return null;
      if (!hasCapturePointPlannerParameters(BalanceManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(BalanceManager.class))
         return null;

      balanceManager = new BalanceManager(controllerToolbox, walkingControllerParameters, capturePointPlannerParameters, angularMomentumModifierParameters,
                                          registry);
      Vector3D linearMomentumWeight = momentumOptimizationSettings.getLinearMomentumWeight();
      Vector3D angularMomentumWeight = momentumOptimizationSettings.getAngularMomentumWeight();
      balanceManager.setMomentumWeight(angularMomentumWeight, linearMomentumWeight);
      balanceManager.setHighMomentumWeightForRecovery(momentumOptimizationSettings.getHighLinearMomentumWeightForRecovery());
      return balanceManager;
   }

   public CenterOfMassHeightManager getOrCreateCenterOfMassHeightManager()
   {
      if (centerOfMassHeightManager != null)
         return centerOfMassHeightManager;

      if (!hasHighLevelHumanoidControllerToolbox(CenterOfMassHeightManager.class))
         return null;
      if (!hasWalkingControllerParameters(CenterOfMassHeightManager.class))
         return null;

      String pelvisName = controllerToolbox.getFullRobotModel().getPelvis().getName();
      Vector3DReadOnly pelvisLinearWeight = taskspaceAngularWeightMap.get(pelvisName);
      centerOfMassHeightManager = new CenterOfMassHeightManager(controllerToolbox, walkingControllerParameters, registry);
      centerOfMassHeightManager.setPelvisTaskspaceWeights(pelvisLinearWeight);
      return centerOfMassHeightManager;
   }

   public RigidBodyControlManager getOrCreateRigidBodyManager(RigidBody bodyToControl, RigidBody baseBody, ReferenceFrame controlFrame,
                                                              ReferenceFrame baseFrame, Collection<ReferenceFrame> trajectoryFrames)
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
      RigidBody elevator = controllerToolbox.getFullRobotModel().getElevator();
      YoDouble yoTime = controllerToolbox.getYoTime();

      ContactablePlaneBody contactableBody = controllerToolbox.getContactableBody(bodyToControl);
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      RigidBodyControlMode defaultControlMode = walkingControllerParameters.getDefaultControlModesForRigidBodies().get(bodyName);

      RigidBodyControlManager manager = new RigidBodyControlManager(bodyToControl, baseBody, elevator, homeConfiguration, homePose, trajectoryFrames,
                                                                    controlFrame, baseFrame, contactableBody, defaultControlMode, yoTime, graphicsListRegistry,
                                                                    registry);
      manager.setGains(jointGainMap, taskspaceOrientationGains, taskspacePositionGains);
      manager.setWeights(jointspaceWeightMap, taskspaceAngularWeight, taskspaceLinearWeight, userModeWeightMap);

      rigidBodyManagerMapByBodyName.put(bodyName, manager);
      return manager;
   }

   public FeetManager getOrCreateFeetManager()
   {
      if (feetManager != null)
         return feetManager;

      if (!hasHighLevelHumanoidControllerToolbox(FeetManager.class))
         return null;
      if (!hasWalkingControllerParameters(FeetManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(FeetManager.class))
         return null;

      feetManager = new FeetManager(controllerToolbox, walkingControllerParameters, registry);
      Vector3D highLinearFootWeight = momentumOptimizationSettings.getHighLinearFootWeight();
      Vector3D highAngularFootWeight = momentumOptimizationSettings.getHighAngularFootWeight();
      Vector3D defaultLinearFootWeight = momentumOptimizationSettings.getDefaultLinearFootWeight();
      Vector3D defaultAngularFootWeight = momentumOptimizationSettings.getDefaultAngularFootWeight();
      feetManager.setWeights(highAngularFootWeight, highLinearFootWeight, defaultAngularFootWeight, defaultLinearFootWeight);
      return feetManager;
   }

   @Deprecated
   public LegConfigurationManager getOrCreateKneeAngleManager()
   {
      return getOrCreateLegConfigurationManager();
   }

   public LegConfigurationManager getOrCreateLegConfigurationManager()
   {
      if (legConfigurationManager != null)
         return legConfigurationManager;

      if (!hasHighLevelHumanoidControllerToolbox(LegConfigurationManager.class))
         return null;
      if (!hasWalkingControllerParameters(LegConfigurationManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(LegConfigurationManager.class))
         return null;

      legConfigurationManager = new LegConfigurationManager(controllerToolbox, walkingControllerParameters, registry);
      return legConfigurationManager;
   }

   public PelvisOrientationManager getOrCreatePelvisOrientationManager()
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
      PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters = walkingControllerParameters.getPelvisOffsetWhileWalkingParameters();
      LeapOfFaithParameters leapOfFaithParameters = walkingControllerParameters.getLeapOfFaithParameters();

      pelvisOrientationManager = new PelvisOrientationManager(pelvisGains, pelvisOffsetWhileWalkingParameters, leapOfFaithParameters, controllerToolbox,
                                                              registry);
      pelvisOrientationManager.setWeights(pelvisAngularWeight);
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

   private boolean hasCapturePointPlannerParameters(Class<?> managerClass)
   {
      if (capturePointPlannerParameters != null)
         return true;
      missingObjectWarning(ICPTrajectoryPlannerParameters.class, managerClass);
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
      PrintTools.warn(this, missingObjectClass.getSimpleName() + " has not been set, cannot create: " + managerClass.getSimpleName());
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

      ret.addCommand(centerOfMassHeightManager.createFeedbackControlTemplate());

      if (pelvisOrientationManager != null)
      {
         ret.addCommand(pelvisOrientationManager.createFeedbackControlTemplate());
      }

      return ret;
   }
}
