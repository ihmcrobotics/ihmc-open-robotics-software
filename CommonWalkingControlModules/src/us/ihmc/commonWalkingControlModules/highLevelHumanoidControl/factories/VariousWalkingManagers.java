package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
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

   public VariousWalkingManagers(HeadOrientationManager headOrientationManager, ChestOrientationManager chestOrientationManager, ManipulationControlModule manipulationControlModule)
   {
      this.headOrientationManager = headOrientationManager;
      this.chestOrientationManager = chestOrientationManager;
      this.manipulationControlModule = manipulationControlModule;
   }

   public static VariousWalkingManagers create(MomentumBasedController momentumBasedController, SideDependentList<HandControllerInterface> handControllers,
         DoubleYoVariable yoTime, VariousWalkingProviders variousWalkingProviders, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();

      DesiredHeadOrientationProvider desiredHeadOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();

      HeadOrientationControlModule headOrientationControlModule = setupHeadOrientationControlModule(fullRobotModel, twistCalculator, 
            registry, dynamicGraphicObjectsListRegistry, momentumBasedController, 
            desiredHeadOrientationProvider, walkingControllerParameters);

      HeadOrientationManager headOrientationManager = new HeadOrientationManager(momentumBasedController, headOrientationControlModule,
                                                         desiredHeadOrientationProvider, registry, dynamicGraphicObjectsListRegistry);


      ChestOrientationControlModule chestOrientationControlModule = setupChestOrientationControlModule(fullRobotModel, 
                                                                       twistCalculator, registry);
      ChestOrientationManager chestOrientationManager = new ChestOrientationManager(momentumBasedController, chestOrientationControlModule);

      // Setup arm+hand manipulation state machines
      ManipulationControlModule manipulationControlModule = new ManipulationControlModule(yoTime, fullRobotModel, twistCalculator, walkingControllerParameters,
                                                               variousWalkingProviders,
                                                               dynamicGraphicObjectsListRegistry, handControllers, momentumBasedController, registry);



      VariousWalkingManagers variousWalkingManagers = new VariousWalkingManagers(headOrientationManager, chestOrientationManager, manipulationControlModule);

      return variousWalkingManagers;
   }

   private static ChestOrientationControlModule setupChestOrientationControlModule(FullRobotModel fullRobotModel,
           TwistCalculator twistCalculator, YoVariableRegistry registry)
   {
      RigidBody chest = fullRobotModel.getChest();
      RigidBody pelvis = fullRobotModel.getPelvis();
      
      ChestOrientationControlModule chestOrientationControlModule = new ChestOrientationControlModule(pelvis, chest, 
                                                                       twistCalculator, registry);

      double chestKp = 100.0;
      double chestZeta = 1.0;
      double chestKd = GainCalculator.computeDerivativeGain(chestKp, chestZeta);

      chestOrientationControlModule.setProportionalGains(chestKp, chestKp, chestKp);
      chestOrientationControlModule.setDerivativeGains(chestKd, chestKd, chestKd);

      return chestOrientationControlModule;
   }
   
   
   private static HeadOrientationControlModule setupHeadOrientationControlModule(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, YoVariableRegistry registry, 
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, 
         MomentumBasedController momentumBasedController, DesiredHeadOrientationProvider desiredHeadOrientationProvider, HeadOrientationControllerParameters headOrientationControllerParameters)
   {      
      
      CommonWalkingReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
     
      RigidBody head = fullRobotModel.getHead();
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody elevator = fullRobotModel.getElevator();
      
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();

      HeadOrientationControlModule headOrientationControlModule = new HeadOrientationControlModule(pelvis, elevator, head, twistCalculator,
            pelvisZUpFrame, chestFrame, headOrientationControllerParameters, registry,
                                                                     dynamicGraphicObjectsListRegistry);

      // Setting initial head pitch
      // This magic number (0.67) is a good default head pitch for getting good LIDAR point coverage of ground by feet
      // it would be in DRC config parameters, but the would require updating several nested constructors with an additional parameter
      FrameOrientation orientation = new FrameOrientation(pelvisZUpFrame, 0.0, 0.67, 0.0);
      headOrientationControlModule.setOrientationToTrack(new FrameOrientation(orientation));
      double headKp = 40.0;
      double headZeta = 1.0;
      double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
      headOrientationControlModule.setProportionalGains(headKp, headKp, headKp);
      headOrientationControlModule.setDerivativeGains(headKd, headKd, headKd);

      if (desiredHeadOrientationProvider != null)
         desiredHeadOrientationProvider.setHeadOrientationControlModule(headOrientationControlModule);

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

}
