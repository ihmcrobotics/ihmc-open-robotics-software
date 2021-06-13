package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

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
                                          copTrajectoryParameters,
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
