//package us.ihmc.darpaRoboticsChallenge.controllers;
//
//import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
//import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
//import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ResizableBipedFoot;
//import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
//import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
//import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
//import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidController;
//import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerFactory;
//import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
//import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
//import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
//import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
//import us.ihmc.projectM.R2Sim02.R2Parameters;
//import us.ihmc.projectM.R2Sim02.output.ProcessedOutputs;
//import us.ihmc.projectM.R2Sim02.sensors.ProcessedSensors;
//import us.ihmc.robotSide.RobotSide;
//import us.ihmc.robotSide.SideDependentList;
//import us.ihmc.utilities.screwTheory.TwistCalculator;
//
//import com.yobotics.simulationconstructionset.DoubleYoVariable;
//import com.yobotics.simulationconstructionset.YoVariableRegistry;
//import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
//import com.yobotics.simulationconstructionset.robotController.RobotController;
//import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
//import com.yobotics.simulationconstructionset.util.ground.steppingStones.SteppingStones;
//
//public class DRCRobotMomentumBasedControllerFactory implements ControllerFactory
//{
//   private final HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
//
//   public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
//   {
//      this.highLevelHumanoidControllerFactory = highLevelHumanoidControllerFactory;
//   }
//
//   public RobotController getController(RobotSpecificJointNames robotJointNames, SteppingStones steppingStones, FullRobotModel fullRobotModel,
//         double controlDT, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GUISetterUpperRegistry guiSetterUpperRegistry,
//         TwistCalculator twistCalculator, SideDependentList<FootSwitchInterface> footSwitches)
//   {
//      double footForward = R2Parameters.FOOT_FORWARD_OFFSET;
//      double footBack = R2Parameters.FOOT_BACKWARD_OFFSET;
//      double footWidth = R2Parameters.FOOT_WIDTH;
//      double footHeight = R2Parameters.FOOT_HEIGHT;
//
//      YoVariableRegistry highLevelControllerRegistry = new YoVariableRegistry("highLevelController");
//      CommonWalkingReferenceFrames referenceFrames = processedSensors.getReferenceFrames();
//
//      double footRotationPreventionFactor = 1.0;
//      double narrowWidthOnToesPercentage = 1.0;
//      double maxToePointsBack = 0.999;
//      double maxHeelPointsForward = 0.999;
//      ResizableBipedFoot rightFoot = ResizableBipedFoot.createRectangularRightFoot(footRotationPreventionFactor, footRotationPreventionFactor, footForward,
//            footBack, footWidth, footHeight, narrowWidthOnToesPercentage, maxToePointsBack, maxHeelPointsForward, referenceFrames, highLevelControllerRegistry,
//            dynamicGraphicObjectsListRegistry);
//      ResizableBipedFoot leftFoot = rightFoot.createLeftFootAsMirrorImage(referenceFrames, highLevelControllerRegistry, dynamicGraphicObjectsListRegistry);
//      SideDependentList<BipedFootInterface> bipedFeet = new SideDependentList<BipedFootInterface>();
//      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(referenceFrames.getAnkleZUpReferenceFrames(), referenceFrames.getMidFeetZUpFrame(),
//            highLevelControllerRegistry, dynamicGraphicObjectsListRegistry);
//
//      bipedFeet.put(RobotSide.RIGHT, rightFoot);
//      bipedFeet.put(RobotSide.LEFT, leftFoot);
//
//      SimpleDesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, highLevelControllerRegistry);
//      desiredHeadingControlModule.setMaxHeadingDot(0.4);
//
//      FullRobotModel fullRobotModel = processedSensors.getFullRobotModel();
//      DoubleYoVariable yoTime = processedSensors.getYoTime();
//      double gravityZ = -processedSensors.getGravityInWorldFrame().getZ();
//      HighLevelHumanoidController highLevelHumanoidController = highLevelHumanoidControllerFactory.create(fullRobotModel, referenceFrames, processedSensors,
//            yoTime, gravityZ, twistCalculator, bipedFeet, bipedSupportPolygons, controlDT, desiredHeadingControlModule, footSwitches,
//            dynamicGraphicObjectsListRegistry, highLevelControllerRegistry, guiSetterUpperRegistry);
//
//      MomentumBasedController momentumBasedController = new MomentumBasedController(processedSensors, processedOutputs, gravityZ, referenceFrames,
//            twistCalculator, controlDT, dynamicGraphicObjectsListRegistry, bipedFeet, bipedSupportPolygons, desiredHeadingControlModule,
//            highLevelHumanoidController);
//      momentumBasedController.getYoVariableRegistry().addChild(highLevelControllerRegistry);
//
//      return momentumBasedController;
//   }
//
//   public int getSimulationTicksPerControlTick()
//   {
//      return 50;
//   }
//}
