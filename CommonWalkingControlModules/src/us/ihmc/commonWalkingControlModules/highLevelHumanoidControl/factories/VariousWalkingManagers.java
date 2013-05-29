package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.head.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.TorusManipulationProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.TorusPoseProvider;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
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

      DesiredHeadOrientationProvider desiredHeadOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();


      HeadOrientationManager headOrientationManager = new HeadOrientationManager(momentumBasedController, walkingControllerParameters,
                                                         desiredHeadOrientationProvider, registry, dynamicGraphicObjectsListRegistry);

      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();

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
      chestOrientationControlModule.setProportionalGains(100.0, 100.0, 100.0);
      chestOrientationControlModule.setDerivativeGains(20.0, 20.0, 20.0);

      return chestOrientationControlModule;
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
