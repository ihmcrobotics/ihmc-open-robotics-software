package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class VariousWalkingManagers
{
   private final HeadOrientationManager headOrientationManager;
   private final ChestOrientationManager chestOrientationManager;
   private final ManipulationControlModule manipulationControlModule;
   private final FeetManager feetManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;

   public VariousWalkingManagers(HeadOrientationManager headOrientationManager, ChestOrientationManager chestOrientationManager,
         ManipulationControlModule manipulationControlModule, FeetManager feetManager, PelvisOrientationManager pelvisOrientationManager, PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager)
   {
      this.headOrientationManager = headOrientationManager;
      this.chestOrientationManager = chestOrientationManager;
      this.manipulationControlModule = manipulationControlModule;
      this.feetManager = feetManager;
      this.pelvisOrientationManager = pelvisOrientationManager;
      this.pelvisICPBasedTranslationManager = pelvisICPBasedTranslationManager;
   }

   public static VariousWalkingManagers create(MomentumBasedController momentumBasedController, VariousWalkingProviders variousWalkingProviders,
         WalkingControllerParameters walkingControllerParameters, ArmControllerParameters armControlParameters, YoVariableRegistry registry,
         DoubleProvider swingTimeProvider)
   {
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      double controlDT = momentumBasedController.getControlDT();

      HeadOrientationProvider desiredHeadOrientationProvider = null;
      HeadOrientationControlModule headOrientationControlModule = null;
      HeadOrientationManager headOrientationManager = null;

      double trajectoryTimeHeadOrientation = walkingControllerParameters.getTrajectoryTimeHeadOrientation();
      if (fullRobotModel.getHead() != null)
      {
         desiredHeadOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();

         headOrientationControlModule = setupHeadOrientationControlModule(momentumBasedController, desiredHeadOrientationProvider, walkingControllerParameters,
               yoGraphicsListRegistry, registry);

         double[] initialHeadYawPitchRoll = walkingControllerParameters.getInitialHeadYawPitchRoll();
         headOrientationManager = new HeadOrientationManager(momentumBasedController, headOrientationControlModule, desiredHeadOrientationProvider,
               trajectoryTimeHeadOrientation, initialHeadYawPitchRoll, registry);
      }

      ChestOrientationProvider desiredChestOrientationProvider = null;
      ChestOrientationManager chestOrientationManager = null;

      if (fullRobotModel.getChest() != null)
      {
         desiredChestOrientationProvider = variousWalkingProviders.getDesiredChestOrientationProvider();
         YoOrientationPIDGains chestControlGains = walkingControllerParameters.createChestControlGains(registry);

         chestOrientationManager = new ChestOrientationManager(momentumBasedController, chestControlGains, desiredChestOrientationProvider, trajectoryTimeHeadOrientation, registry);
      }

      ManipulationControlModule manipulationControlModule = null;

      if (fullRobotModel.getChest() != null && fullRobotModel.getHand(RobotSide.LEFT) != null && fullRobotModel.getHand(RobotSide.RIGHT) != null)
      {
         // Setup arm+hand manipulation state machines
         manipulationControlModule = new ManipulationControlModule(variousWalkingProviders, armControlParameters, momentumBasedController, registry);
      }

      FeetManager feetManager = new FeetManager(momentumBasedController, walkingControllerParameters, swingTimeProvider, registry);

      PelvisPoseProvider desiredPelvisPoseProvider = variousWalkingProviders.getDesiredPelvisPoseProvider();
      PelvisOrientationManager pelvisOrientationManager = new PelvisOrientationManager(walkingControllerParameters, momentumBasedController, desiredPelvisPoseProvider, registry);
      
      YoPDGains pelvisXYControlGains = walkingControllerParameters.createPelvisICPBasedXYControlGains(registry);
      PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(momentumBasedController, desiredPelvisPoseProvider, pelvisXYControlGains, registry);

      VariousWalkingManagers variousWalkingManagers = new VariousWalkingManagers(headOrientationManager, chestOrientationManager, manipulationControlModule,
            feetManager, pelvisOrientationManager, pelvisICPBasedTranslationManager);

      return variousWalkingManagers;
   }

   private static HeadOrientationControlModule setupHeadOrientationControlModule(MomentumBasedController momentumBasedController,
         HeadOrientationProvider desiredHeadOrientationProvider, HeadOrientationControllerParameters headOrientationControllerParameters,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();

      ReferenceFrame headOrientationExpressedInFrame;
      if (desiredHeadOrientationProvider != null)
         headOrientationExpressedInFrame = desiredHeadOrientationProvider.getHeadOrientationExpressedInFrame();
      else
         headOrientationExpressedInFrame = referenceFrames.getPelvisZUpFrame(); // ReferenceFrame.getWorldFrame(); //

      YoOrientationPIDGains gains = headOrientationControllerParameters.createHeadOrientationControlGains(registry);
      HeadOrientationControlModule headOrientationControlModule = new HeadOrientationControlModule(momentumBasedController, headOrientationExpressedInFrame,
            headOrientationControllerParameters, gains, registry, yoGraphicsListRegistry);

      return headOrientationControlModule;
   }

   public void initializeManagers()
   {
      if (manipulationControlModule != null)
         manipulationControlModule.initialize();
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
}
