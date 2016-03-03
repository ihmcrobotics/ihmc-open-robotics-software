package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;

public class VariousWalkingManagers
{
   private final HeadOrientationManager headOrientationManager;
   private final ChestOrientationManager chestOrientationManager;
   private final ManipulationControlModule manipulationControlModule;
   private final FeetManager feetManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;

   public VariousWalkingManagers(HeadOrientationManager headOrientationManager, ChestOrientationManager chestOrientationManager,
         ManipulationControlModule manipulationControlModule, FeetManager feetManager, PelvisOrientationManager pelvisOrientationManager,
         PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager)
   {
      this.headOrientationManager = headOrientationManager;
      this.chestOrientationManager = chestOrientationManager;
      this.manipulationControlModule = manipulationControlModule;
      this.feetManager = feetManager;
      this.pelvisOrientationManager = pelvisOrientationManager;
      this.pelvisICPBasedTranslationManager = pelvisICPBasedTranslationManager;
   }

   public static VariousWalkingManagers create(MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         ArmControllerParameters armControlParameters, YoVariableRegistry registry)
   {
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();

      HeadOrientationManager headOrientationManager = null;
      if (fullRobotModel.getHead() != null)
      {
         YoOrientationPIDGainsInterface headControlGains = walkingControllerParameters.createHeadOrientationControlGains(registry);
         double[] initialHeadYawPitchRoll = walkingControllerParameters.getInitialHeadYawPitchRoll();
         headOrientationManager = new HeadOrientationManager(momentumBasedController, walkingControllerParameters, headControlGains, initialHeadYawPitchRoll,
               registry);
      }

      ChestOrientationManager chestOrientationManager = null;
      if (fullRobotModel.getChest() != null)
      {
         double trajectoryTimeHeadOrientation = walkingControllerParameters.getTrajectoryTimeHeadOrientation();
         YoOrientationPIDGainsInterface chestControlGains = walkingControllerParameters.createChestControlGains(registry);
         chestOrientationManager = new ChestOrientationManager(momentumBasedController, chestControlGains, trajectoryTimeHeadOrientation, registry);
      }

      ManipulationControlModule manipulationControlModule = null;

      if (fullRobotModel.getChest() != null && fullRobotModel.getHand(RobotSide.LEFT) != null && fullRobotModel.getHand(RobotSide.RIGHT) != null)
      {
         // Setup arm+hand manipulation state machines
         manipulationControlModule = new ManipulationControlModule(armControlParameters, momentumBasedController, registry);
      }

      FeetManager feetManager = new FeetManager(momentumBasedController, walkingControllerParameters, registry);

      PelvisOrientationManager pelvisOrientationManager = new PelvisOrientationManager(walkingControllerParameters, momentumBasedController, registry);

      YoPDGains pelvisXYControlGains = walkingControllerParameters.createPelvisICPBasedXYControlGains(registry);
      PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(momentumBasedController, pelvisXYControlGains,
            registry);

      VariousWalkingManagers variousWalkingManagers = new VariousWalkingManagers(headOrientationManager, chestOrientationManager, manipulationControlModule,
            feetManager, pelvisOrientationManager, pelvisICPBasedTranslationManager);

      return variousWalkingManagers;
   }

   public void initializeManagers()
   {
      if (manipulationControlModule != null)
         manipulationControlModule.initialize();
      if (headOrientationManager != null)
         headOrientationManager.initialize();
      if (chestOrientationManager != null)
         chestOrientationManager.initialize();
   }

   public HeadOrientationManager getHeadOrientationManager()
   {
      return headOrientationManager;
   }

   public ChestOrientationManager getChestOrientationManager()
   {
      return chestOrientationManager;
   }

   public ManipulationControlModule getManipulationControlModule()
   {
      return manipulationControlModule;
   }

   public FeetManager getFeetManager()
   {
      return feetManager;
   }

   public PelvisOrientationManager getPelvisOrientationManager()
   {
      return pelvisOrientationManager;
   }

   public PelvisICPBasedTranslationManager getPelvisICPBasedTranslationManager()
   {
      return pelvisICPBasedTranslationManager;
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
         ret.addCommand(headOrientationManager.getFeedbackControlCommand());
      }

      if (chestOrientationManager != null)
      {
         ret.addCommand(chestOrientationManager.getFeedbackControlCommand());
      }

      if (pelvisOrientationManager != null)
      {
         ret.addCommand(pelvisOrientationManager.getFeedbackControlCommand());
      }

      return ret;
   }
}
