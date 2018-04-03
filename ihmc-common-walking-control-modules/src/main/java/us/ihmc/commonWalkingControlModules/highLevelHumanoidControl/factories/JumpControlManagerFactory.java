package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.AbstractHighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ParameterTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.flight.CentroidalMomentumManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.FeetJumpManager;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpControlManagerInterface;
import us.ihmc.commonWalkingControlModules.controlModules.flight.PelvisControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JumpControlManagerFactory extends AbstractHighLevelControllerParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableRegistry momentumRegistry = new YoVariableRegistry("MomentumRegistry");

   private final JumpControllerParameters jumpControllerParameters;
   private final MomentumOptimizationSettings momentumOptimizationSettings;
   private HighLevelHumanoidControllerToolbox controllerToolbox;

   private FeetJumpManager feetManager;
   private CentroidalMomentumManager momentumManager;
   private PelvisControlManager pelvisControlManager;

   private final List<JumpControlManagerInterface> controlManagers = new ArrayList<>();
   private final Map<String, RigidBodyControlManager> rigidBodyManagerMapByBodyName = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspaceOrientationGainMap = new HashMap<>();
   private final Map<String, PID3DGainsReadOnly> taskspacePositionGainMap = new HashMap<>();
   private final Map<String, Vector3DReadOnly> taskspaceAngularWeightMap = new HashMap<>();
   private final Map<String, Vector3DReadOnly> taskspaceLinearWeightMap = new HashMap<>();
   private final Map<String, DoubleProvider> jointspaceWeightMap = new HashMap<>();
   private final Map<String, PIDGainsReadOnly> jointGainMap = new HashMap<>();
   private final Map<String, DoubleProvider> userModeWeightMap = new HashMap<>();

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   public JumpControlManagerFactory(JumpControllerParameters jumpControllerParameters, YoVariableRegistry parentRegistry)
   {
      this.jumpControllerParameters = jumpControllerParameters;
      this.momentumOptimizationSettings = jumpControllerParameters.getMomentumOptimizationSettings();

      ParameterTools.extractJointGainMap(jumpControllerParameters.getJointSpaceControlGains(), jointGainMap, registry);
      ParameterTools.extract3DGainMap("Orientation", jumpControllerParameters.getTaskspaceOrientationControlGains(), taskspaceOrientationGainMap, registry);
      ParameterTools.extract3DGainMap("Position", jumpControllerParameters.getTaskspacePositionControlGains(), taskspacePositionGainMap, registry);
      ParameterTools.extractJointWeightMap("JointspaceWeight", momentumOptimizationSettings.getJointspaceWeights(), jointspaceWeightMap, momentumRegistry);
      ParameterTools.extractJointWeightMap("UserModeWeight", momentumOptimizationSettings.getUserModeWeights(), userModeWeightMap, momentumRegistry);
      ParameterTools.extract3DWeightMap("AngularWeight", momentumOptimizationSettings.getTaskspaceAngularWeights(), taskspaceAngularWeightMap,
                                        momentumRegistry);
      ParameterTools.extract3DWeightMap("LinearWeight", momentumOptimizationSettings.getTaskspaceLinearWeights(), taskspaceLinearWeightMap, momentumRegistry);

      parentRegistry.addChild(momentumRegistry);
      parentRegistry.addChild(registry);
   }

   public PelvisControlManager getOrCreatePelvisControlManager()
   {
      if (pelvisControlManager != null)
         return pelvisControlManager;

      if (!hasHighLevelHumanoidControllerToolbox(PelvisControlManager.class))
         return null;
      if (!hasJumpControllerParameters(PelvisControlManager.class))
         return null;

      String bodyName = controllerToolbox.getFullRobotModel().getPelvis().getName();
      PID3DGainsReadOnly positionGains = taskspacePositionGainMap.get(bodyName);
      PID3DGainsReadOnly orientationGains = taskspaceOrientationGainMap.get(bodyName);
      Vector3DReadOnly positionWeights = taskspaceLinearWeightMap.get(bodyName);
      Vector3DReadOnly orientationWeights = taskspaceAngularWeightMap.get(bodyName);

      pelvisControlManager = new PelvisControlManager(controllerToolbox, registry);
      pelvisControlManager.setGains(positionGains, orientationGains);
      pelvisControlManager.setWeights(positionWeights, orientationWeights);
      controlManagers.add(pelvisControlManager);
      return pelvisControlManager;
   }

   public FeetJumpManager getOrCreateFeetManager()
   {
      if (feetManager != null)
         return feetManager;

      if (!hasHighLevelHumanoidControllerToolbox(FeetManager.class))
         return null;
      if (!hasJumpControllerParameters(FeetManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(FeetManager.class))
         return null;
      TObjectDoubleHashMap<String> homeConfiguration = jumpControllerParameters.getOrCreateJointHomeConfiguration();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();

      feetManager = new FeetJumpManager(controllerToolbox, jumpControllerParameters, trajectoryFrames, homeConfiguration, graphicsListRegistry, registry);

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         String bodyName = fullRobotModel.getFoot(robotSide).getName();
         PID3DGainsReadOnly taskspaceOrientationGains = taskspaceOrientationGainMap.get(bodyName);
         PID3DGainsReadOnly taskspacePositionGains = taskspacePositionGainMap.get(bodyName);

         Vector3DReadOnly taskspaceAngularWeight = taskspaceAngularWeightMap.get(bodyName);
         Vector3DReadOnly taskspaceLinearWeight = taskspaceLinearWeightMap.get(bodyName);
         feetManager.setGains(robotSide, jointGainMap, taskspaceOrientationGains, taskspacePositionGains, taskspaceOrientationGains,
                              taskspacePositionGains);
         feetManager.setWeights(robotSide, jointspaceWeightMap, taskspaceAngularWeight, taskspaceLinearWeight, taskspaceAngularWeight,
                                taskspaceLinearWeight);
      }
      controlManagers.add(feetManager);
      return feetManager;
   }

   public CentroidalMomentumManager getOrCreateCentroidalMomentumManager()
   {
      if (momentumManager != null)
         return momentumManager;

      if (!hasHighLevelHumanoidControllerToolbox(CentroidalMomentumManager.class))
         return null;

      momentumManager = new CentroidalMomentumManager(controllerToolbox, jumpControllerParameters, registry);
      controlManagers.add(momentumManager);
      return momentumManager;
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

      if (!hasJumpControllerParameters(RigidBodyControlManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(RigidBodyControlManager.class))
         return null;
      if (!hasHighLevelHumanoidControllerToolbox(RigidBodyControlManager.class))
         return null;

      // Gains
      PID3DGainsReadOnly taskspaceOrientationGains = taskspaceOrientationGainMap.get(bodyName);
      PID3DGainsReadOnly taskspacePositionGains = taskspacePositionGainMap.get(bodyName);

      // Weights
      Vector3DReadOnly taskspaceAngularWeight = taskspaceAngularWeightMap.get(bodyName);
      Vector3DReadOnly taskspaceLinearWeight = taskspaceLinearWeightMap.get(bodyName);

      TObjectDoubleHashMap<String> homeConfiguration = jumpControllerParameters.getOrCreateJointHomeConfiguration();
      Pose3D homePose = jumpControllerParameters.getOrCreateBodyHomeConfiguration().get(bodyName);
      RigidBody elevator = controllerToolbox.getFullRobotModel().getElevator();
      YoDouble yoTime = controllerToolbox.getYoTime();

      ContactablePlaneBody contactableBody = controllerToolbox.getContactableBody(bodyToControl);
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      RigidBodyControlMode defaultControlMode = jumpControllerParameters.getDefaultControlModesForRigidBodies().get(bodyName);

      RigidBodyControlManager manager = new RigidBodyControlManager(bodyToControl, baseBody, elevator, homeConfiguration, homePose, trajectoryFrames,
                                                                    controlFrame, baseFrame, contactableBody, defaultControlMode, yoTime, graphicsListRegistry,
                                                                    registry);
      manager.setGains(jointGainMap, taskspaceOrientationGains, taskspacePositionGains);
      manager.setWeights(jointspaceWeightMap, taskspaceAngularWeight, taskspaceLinearWeight, userModeWeightMap);

      rigidBodyManagerMapByBodyName.put(bodyName, manager);
      return manager;
   }

   private boolean hasHighLevelHumanoidControllerToolbox(Class<?> managerClass)
   {
      if (controllerToolbox != null)
         return true;
      missingObjectWarning(HighLevelHumanoidControllerToolbox.class, managerClass);
      return false;
   }

   private boolean hasJumpControllerParameters(Class<?> managerClass)
   {
      if (jumpControllerParameters != null)
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
      PrintTools.warn(this, missingObjectClass.getSimpleName() + " has not been set, cannot create: " + managerClass.getSimpleName());
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList templateFeedbackCommandList = new FeedbackControlCommandList();
      Collection<RigidBodyControlManager> bodyManagers = rigidBodyManagerMapByBodyName.values();
      for (RigidBodyControlManager bodyManager : bodyManagers)
      {
         if (bodyManager != null)
            templateFeedbackCommandList.addCommand(bodyManager.createFeedbackControlTemplate());
      }
      for (JumpControlManagerInterface controlManager : controlManagers)
      {
         FeedbackControlCommand<?> feedbackControlCommand = controlManager.createFeedbackControlTemplate();
         if (feedbackControlCommand != null)
         {
            templateFeedbackCommandList.addCommand(feedbackControlCommand);
         }
      }
      return templateFeedbackCommandList;
   }

   public void setHighLevelHumanoidControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
   }
}
