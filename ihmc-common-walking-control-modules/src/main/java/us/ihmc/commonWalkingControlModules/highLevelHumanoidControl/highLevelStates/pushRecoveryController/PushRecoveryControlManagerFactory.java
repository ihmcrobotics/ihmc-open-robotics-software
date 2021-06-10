package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.DefaultSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
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
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDSE3Gains;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class PushRecoveryControlManagerFactory
{
   public static final String weightRegistryName = "PushRecoveryOptimizationSettings";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final HighLevelControlManagerFactory controlManagerFactory;

   private PushRecoveryBalanceManager balanceManager;

   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private WalkingControllerParameters walkingControllerParameters;
   private PushRecoveryControllerParameters pushRecoveryControllerParameters;
   private CoPTrajectoryParameters copTrajectoryParameters;
   private SplitFractionCalculatorParametersReadOnly splitFractionParameters = new DefaultSplitFractionCalculatorParameters();
   private MomentumOptimizationSettings momentumOptimizationSettings;


   public PushRecoveryControlManagerFactory(HighLevelControlManagerFactory controlManagerFactory,
                                            YoRegistry parentRegistry)
   {
      this.controlManagerFactory = controlManagerFactory;
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
   }

   public void setPushRecoveryControllerParameters(PushRecoveryControllerParameters pushRecoveryControllerParameters)
   {
      this.pushRecoveryControllerParameters = pushRecoveryControllerParameters;
   }

   public void setCopTrajectoryParameters(CoPTrajectoryParameters copTrajectoryParameters)
   {
      this.copTrajectoryParameters = copTrajectoryParameters;
   }

   public void setSplitFractionParameters(SplitFractionCalculatorParametersReadOnly splitFractionParameters)
   {
      this.splitFractionParameters = splitFractionParameters;
   }

   public PushRecoveryBalanceManager getOrCreateBalanceManager()
   {
      if (balanceManager != null)
         return balanceManager;

      if (!hasHighLevelHumanoidControllerToolbox(BalanceManager.class))
         return null;
      if (!hasWalkingControllerParameters(BalanceManager.class))
         return null;
      if (!hasPushRecoveryControllerParameters(BalanceManager.class))
         return null;
      if (!hasCoPTrajectoryParameters(BalanceManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(BalanceManager.class))
         return null;

      balanceManager = new PushRecoveryBalanceManager(controllerToolbox,
                                          walkingControllerParameters,
                                          copTrajectoryParameters,
                                          splitFractionParameters,
                                          registry);
      return balanceManager;
   }

   public CenterOfMassHeightManager getOrCreateCenterOfMassHeightManager()
   {
      return controlManagerFactory.getOrCreateCenterOfMassHeightManager();
   }

   public RigidBodyControlManager getOrCreateRigidBodyManager(RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, ReferenceFrame controlFrame,
                                                              ReferenceFrame baseFrame)
   {
      return controlManagerFactory.getOrCreateRigidBodyManager(bodyToControl, baseBody, controlFrame, baseFrame);
   }

   public FeetManager getOrCreateFeetManager()
   {
      return controlManagerFactory.getOrCreateFeetManager();
   }


   public LegConfigurationManager getOrCreateLegConfigurationManager()
   {
      return controlManagerFactory.getOrCreateLegConfigurationManager();
   }

   public PelvisOrientationManager getOrCreatePelvisOrientationManager()
   {
      return controlManagerFactory.getOrCreatePelvisOrientationManager();
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

   private boolean hasPushRecoveryControllerParameters(Class<?> managerClass)
   {
      if (pushRecoveryControllerParameters != null)
         return true;
      missingObjectWarning(PushRecoveryControllerParameters.class, managerClass);
      return false;
   }

   private boolean hasCoPTrajectoryParameters(Class<?> managerClass)
   {
      if (copTrajectoryParameters != null)
         return true;
      missingObjectWarning(CoPTrajectoryParameters.class, managerClass);
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
      controlManagerFactory.initializeManagers();
   }

   public FeedbackControllerTemplate createFeedbackControlTemplate()
   {
      return controlManagerFactory.createFeedbackControlTemplate();
   }
}
