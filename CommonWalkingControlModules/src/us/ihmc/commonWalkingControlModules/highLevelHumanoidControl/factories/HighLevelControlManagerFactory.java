package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.chest.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class HighLevelControlManagerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StatusMessageOutputManager statusOutputManager;

   private BalanceManager balanceManager;
   private CenterOfMassHeightManager centerOfMassHeightManager;
   private HeadOrientationManager headOrientationManager;
   private ChestOrientationManager chestOrientationManager;
   private ManipulationControlModule manipulationControlModule;
   private FeetManager feetManager;
   private PelvisOrientationManager pelvisOrientationManager;

   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private WalkingControllerParameters walkingControllerParameters;
   private CapturePointPlannerParameters capturePointPlannerParameters;
   private ICPOptimizationParameters icpOptimizationParameters;
   private ArmControllerParameters armControllerParameters;
   private MomentumOptimizationSettings momentumOptimizationSettings;

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
   }

   public void setCapturePointPlannerParameters(CapturePointPlannerParameters capturePointPlannerParameters)
   {
      this.capturePointPlannerParameters = capturePointPlannerParameters;
   }

   public void setICPOptimizationParameters(ICPOptimizationParameters icpOptimizationParameters)
   {
      this.icpOptimizationParameters = icpOptimizationParameters;
   }

   public void setArmControlParameters(ArmControllerParameters armControllerParameters)
   {
      this.armControllerParameters = armControllerParameters;
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

      balanceManager = new BalanceManager(controllerToolbox, walkingControllerParameters, capturePointPlannerParameters, icpOptimizationParameters, registry);
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

      centerOfMassHeightManager = new CenterOfMassHeightManager(controllerToolbox, walkingControllerParameters, registry);
      return centerOfMassHeightManager;
   }

   public HeadOrientationManager getOrCreatedHeadOrientationManager()
   {
      if (headOrientationManager != null)
         return headOrientationManager;

      FullRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      if (fullRobotModel.getHead() == null)
      {
         robotMissingBodyWarning("head", HeadOrientationManager.class);
         return null;
      }

      if (!hasWalkingControllerParameters(HeadOrientationManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(HeadOrientationManager.class))
         return null;

      headOrientationManager = new HeadOrientationManager(controllerToolbox, walkingControllerParameters, registry);
      double headJointspaceWeight = momentumOptimizationSettings.getHeadJointspaceWeight();
      double headTaskspaceWeight = momentumOptimizationSettings.getHeadTaskspaceWeight();
      double headUserModeWeight = momentumOptimizationSettings.getHeadUserModeWeight();
      headOrientationManager.setWeights(headJointspaceWeight, headTaskspaceWeight, headUserModeWeight);
      return headOrientationManager;
   }

   public ChestOrientationManager getOrCreateChestOrientationManager()
   {
      if (chestOrientationManager != null)
         return chestOrientationManager;

      FullRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      if (fullRobotModel.getChest() == null)
      {
         robotMissingBodyWarning("chest", ChestOrientationManager.class);
         return null;
      }

      if (!hasWalkingControllerParameters(ChestOrientationManager.class))
         return null;
      if (!hasMomentumOptimizationSettings(ChestOrientationManager.class))
         return null;

      Vector3D chestAngularWeight = momentumOptimizationSettings.getChestAngularWeight();
      chestOrientationManager = new ChestOrientationManager(controllerToolbox, walkingControllerParameters, registry);
      return chestOrientationManager;
   }

   public ManipulationControlModule getOrCreateManipulationControlModule()
   {
      if (manipulationControlModule != null)
         return manipulationControlModule;

      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();

      if (fullRobotModel.getChest() == null)
      {
         robotMissingBodyWarning("chest", ManipulationControlModule.class);
         return null;
      }

      if (fullRobotModel.getHand(RobotSide.LEFT) == null)
      {
         robotMissingBodyWarning("left hand", ManipulationControlModule.class);
         return null;
      }

      if (fullRobotModel.getHand(RobotSide.RIGHT) == null)
      {
         robotMissingBodyWarning("right hand", ManipulationControlModule.class);
         return null;
      }

      if (!hasArmControllerParameters(ManipulationControlModule.class))
         return null;
      if (!hasHighLevelHumanoidControllerToolbox(ManipulationControlModule.class))
         return null;
      if (!hasMomentumOptimizationSettings(ManipulationControlModule.class))
         return null;

      manipulationControlModule = new ManipulationControlModule(armControllerParameters, controllerToolbox, registry);
      double handJointspaceWeight = momentumOptimizationSettings.getHandJointspaceWeight();
      Vector3D handAngularTaskspaceWeight = momentumOptimizationSettings.getHandAngularTaskspaceWeight();
      Vector3D handLinearTaskspaceWeight = momentumOptimizationSettings.getHandLinearTaskspaceWeight();
      double handUserModeWeight = momentumOptimizationSettings.getHandUserModeWeight();
      manipulationControlModule.setWeights(handJointspaceWeight, handAngularTaskspaceWeight, handLinearTaskspaceWeight, handUserModeWeight);
      return manipulationControlModule;
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

      pelvisOrientationManager = new PelvisOrientationManager(walkingControllerParameters, controllerToolbox, registry);
      pelvisOrientationManager.setWeights(momentumOptimizationSettings.getPelvisAngularWeight());
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
      missingObjectWarning(CapturePointPlannerParameters.class, managerClass);
      return false;
   }

   private boolean hasArmControllerParameters(Class<?> managerClass)
   {
      if (armControllerParameters != null)
         return true;
      missingObjectWarning(ArmControllerParameters.class, managerClass);
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

   private void robotMissingBodyWarning(String missingBodyName, Class<?> managerClass)
   {
      PrintTools.warn(this, "The robot is missing the body: " + missingBodyName + ", cannot create: " + managerClass.getSimpleName());
   }

   public void initializeManagers()
   {
      if (balanceManager != null)
         balanceManager.initialize();
      if (centerOfMassHeightManager != null)
         centerOfMassHeightManager.initialize();
      if (manipulationControlModule != null)
         manipulationControlModule.initialize();
      if (headOrientationManager != null)
         headOrientationManager.initialize();
      if (chestOrientationManager != null)
         chestOrientationManager.initialize();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();

      if (manipulationControlModule != null)
      {
         FeedbackControlCommandList template = manipulationControlModule.createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }

      if (feetManager != null)
      {
         FeedbackControlCommandList template = feetManager.createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }

      if (headOrientationManager != null)
      {
         ret.addCommand(headOrientationManager.createFeedbackControlTemplate());
      }

      if (chestOrientationManager != null)
      {
         ret.addCommand(chestOrientationManager.createFeedbackControlTemplate());
      }

      if (pelvisOrientationManager != null)
      {
         ret.addCommand(pelvisOrientationManager.getFeedbackControlCommand());
      }

      return ret;
   }
}
