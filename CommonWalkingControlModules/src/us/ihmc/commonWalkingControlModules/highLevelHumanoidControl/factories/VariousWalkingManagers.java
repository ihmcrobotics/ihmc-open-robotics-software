package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisDesiredsHandler;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class VariousWalkingManagers
{
   private final HeadOrientationManager headOrientationManager;
   private final ChestOrientationManager chestOrientationManager;
   private final ManipulationControlModule manipulationControlModule;
   private final PelvisDesiredsHandler pelvisDesiredsHandler;    // mid competition hack

   public VariousWalkingManagers(HeadOrientationManager headOrientationManager, ChestOrientationManager chestOrientationManager,
                                 ManipulationControlModule manipulationControlModule, PelvisDesiredsHandler pelvisDesiredsHandler)
   {
      this.headOrientationManager = headOrientationManager;
      this.chestOrientationManager = chestOrientationManager;
      this.manipulationControlModule = manipulationControlModule;
      this.pelvisDesiredsHandler = pelvisDesiredsHandler;
   }

   public static VariousWalkingManagers create(MomentumBasedController momentumBasedController, SideDependentList<HandControllerInterface> handControllers,
           DoubleYoVariable yoTime, VariousWalkingProviders variousWalkingProviders, WalkingControllerParameters walkingControllerParameters,
           YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      double controlDT = momentumBasedController.getControlDT();

      DesiredHeadOrientationProvider desiredHeadOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();

      HeadOrientationControlModule headOrientationControlModule = setupHeadOrientationControlModule(fullRobotModel, twistCalculator, registry,
                                                                     dynamicGraphicObjectsListRegistry, momentumBasedController,
                                                                     desiredHeadOrientationProvider, walkingControllerParameters);

      HeadOrientationManager headOrientationManager = new HeadOrientationManager(momentumBasedController, headOrientationControlModule,
                                                         desiredHeadOrientationProvider);


      ChestOrientationControlModule chestOrientationControlModule = setupChestOrientationControlModule(fullRobotModel, twistCalculator, registry);
      ChestOrientationManager chestOrientationManager = new ChestOrientationManager(momentumBasedController, chestOrientationControlModule);

      // Setup arm+hand manipulation state machines
      ManipulationControlModule manipulationControlModule = new ManipulationControlModule(yoTime, fullRobotModel, twistCalculator, walkingControllerParameters,
                                                               variousWalkingProviders, dynamicGraphicObjectsListRegistry, handControllers,
                                                               momentumBasedController, registry);

      PelvisDesiredsHandler pelvisDesiredsHandler = new PelvisDesiredsHandler(controlDT, yoTime, registry);


      VariousWalkingManagers variousWalkingManagers = new VariousWalkingManagers(headOrientationManager, chestOrientationManager, manipulationControlModule,
            pelvisDesiredsHandler);

      return variousWalkingManagers;
   }

   private static ChestOrientationControlModule setupChestOrientationControlModule(FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
           YoVariableRegistry registry)
   {
      RigidBody chest = fullRobotModel.getChest();
      RigidBody pelvis = fullRobotModel.getPelvis();

      ChestOrientationControlModule chestOrientationControlModule = new ChestOrientationControlModule(pelvis, chest, twistCalculator, registry);

      return chestOrientationControlModule;
   }


   private static HeadOrientationControlModule setupHeadOrientationControlModule(FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
           YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, MomentumBasedController momentumBasedController,
           DesiredHeadOrientationProvider desiredHeadOrientationProvider, HeadOrientationControllerParameters headOrientationControllerParameters)
   {
      CommonWalkingReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();

      RigidBody head = fullRobotModel.getHead();
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody elevator = fullRobotModel.getElevator();

      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();

      ReferenceFrame headOrientationExpressedInFrame;
      if (desiredHeadOrientationProvider != null)
         headOrientationExpressedInFrame = desiredHeadOrientationProvider.getHeadOrientationExpressedInFrame();
      else
         headOrientationExpressedInFrame = referenceFrames.getPelvisZUpFrame();
      HeadOrientationControlModule headOrientationControlModule = new HeadOrientationControlModule(pelvis, elevator, head, twistCalculator,
                                                                     headOrientationExpressedInFrame, chestFrame, headOrientationControllerParameters,
                                                                     registry, dynamicGraphicObjectsListRegistry);

      // Setting initial head pitch
      // This magic number (0.67) is a good default head pitch for getting good LIDAR point coverage of ground by feet
      // it would be in DRC config parameters, but the would require updating several nested constructors with an additional parameter
      FrameOrientation orientation = new FrameOrientation(headOrientationExpressedInFrame, 0.0, 0.67, 0.0);
      headOrientationControlModule.setOrientationToTrack(new FrameOrientation(orientation));

      return headOrientationControlModule;
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

   public PelvisDesiredsHandler getPelvisDesiredsHandler()
   {
      return pelvisDesiredsHandler;
   }
}
