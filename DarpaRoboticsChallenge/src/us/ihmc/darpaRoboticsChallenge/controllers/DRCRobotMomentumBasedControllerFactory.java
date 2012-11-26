package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ResizableBipedFoot;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.steppingStones.SteppingStones;

public class DRCRobotMomentumBasedControllerFactory implements ControllerFactory
{
   private final HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;

   public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
   {
      this.highLevelHumanoidControllerFactory = highLevelHumanoidControllerFactory;
   }

   public RobotController getController(FullRobotModel fullRobotModel, ReferenceFrames referenceFrames, SteppingStones steppingStones,
         double controlDT, DoubleYoVariable yoTime, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GUISetterUpperRegistry guiSetterUpperRegistry,
         TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian, SideDependentList<FootSwitchInterface> footSwitches)
   {
      double footForward = 0.17;
      double footBack = 0.07;
      double footWidth = 0.12;
      double footHeight = 0.084;

      YoVariableRegistry highLevelControllerRegistry = new YoVariableRegistry("highLevelController");
      double footRotationPreventionFactor = 1.0;
      double narrowWidthOnToesPercentage = 1.0;
      double maxToePointsBack = 0.999;
      double maxHeelPointsForward = 0.999;
      ResizableBipedFoot rightFoot = ResizableBipedFoot.createRectangularRightFoot(footRotationPreventionFactor, footRotationPreventionFactor, footForward,
            footBack, footWidth, footHeight, narrowWidthOnToesPercentage, maxToePointsBack, maxHeelPointsForward, referenceFrames, highLevelControllerRegistry,
            dynamicGraphicObjectsListRegistry);
      ResizableBipedFoot leftFoot = rightFoot.createLeftFootAsMirrorImage(referenceFrames, highLevelControllerRegistry, dynamicGraphicObjectsListRegistry);
      SideDependentList<BipedFootInterface> bipedFeet = new SideDependentList<BipedFootInterface>();
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(referenceFrames.getAnkleZUpReferenceFrames(), referenceFrames.getMidFeetZUpFrame(),
            highLevelControllerRegistry, dynamicGraphicObjectsListRegistry);

      bipedFeet.put(RobotSide.RIGHT, rightFoot);
      bipedFeet.put(RobotSide.LEFT, leftFoot);

      SimpleDesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, highLevelControllerRegistry);
      desiredHeadingControlModule.setMaxHeadingDot(0.4);
      desiredHeadingControlModule.updateDesiredHeadingFrame();
      
      ManualDesiredVelocityControlModule desiredVelocityControlModule = new ManualDesiredVelocityControlModule(highLevelControllerRegistry);
      desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(ReferenceFrame.getWorldFrame(), 1.0, 0.0));

      double gravityZ = 9.81;
      HighLevelHumanoidController highLevelHumanoidController = highLevelHumanoidControllerFactory.create(fullRobotModel, referenceFrames, 
            null, yoTime, gravityZ, twistCalculator, centerOfMassJacobian, bipedFeet, bipedSupportPolygons, controlDT, desiredHeadingControlModule, desiredVelocityControlModule,
            footSwitches, dynamicGraphicObjectsListRegistry, highLevelControllerRegistry, guiSetterUpperRegistry);

      MomentumBasedController momentumBasedController = new MomentumBasedController(fullRobotModel, null, gravityZ, referenceFrames,
            twistCalculator, controlDT, dynamicGraphicObjectsListRegistry, bipedFeet, bipedSupportPolygons, highLevelHumanoidController);
      momentumBasedController.getYoVariableRegistry().addChild(highLevelControllerRegistry);

      return momentumBasedController;
   }

   public int getSimulationTicksPerControlTick()
   {
      return 50;
   }
}
