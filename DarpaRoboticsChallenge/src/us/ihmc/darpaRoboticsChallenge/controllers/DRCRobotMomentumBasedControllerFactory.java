package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
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

   public RobotController getController(FullRobotModel fullRobotModel, ReferenceFrames referenceFrames, SteppingStones steppingStones, double controlDT,
           DoubleYoVariable yoTime, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GUISetterUpperRegistry guiSetterUpperRegistry,
           TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian, SideDependentList<FootSwitchInterface> footSwitches)
   {
      double footForward = 0.17;
      double footBack = 0.07;
      double footWidth = 0.12;

      YoVariableRegistry highLevelControllerRegistry = new YoVariableRegistry("highLevelController");
      
      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;
         
         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(referenceFrames.getAnkleZUpReferenceFrames(), referenceFrames.getMidFeetZUpFrame(),
            highLevelControllerRegistry, dynamicGraphicObjectsListRegistry);

      double gravityZ = 9.81;

      HighLevelHumanoidController highLevelHumanoidController = highLevelHumanoidControllerFactory.create(fullRobotModel, referenceFrames, null, yoTime,
                                                                   gravityZ, twistCalculator, centerOfMassJacobian, bipedFeet, bipedSupportPolygons, controlDT,
                                                                   footSwitches, dynamicGraphicObjectsListRegistry, highLevelControllerRegistry,
                                                                   guiSetterUpperRegistry);

      MomentumBasedController momentumBasedController = new MomentumBasedController(fullRobotModel, null, gravityZ, referenceFrames, twistCalculator,
                                                           controlDT, dynamicGraphicObjectsListRegistry, bipedSupportPolygons, highLevelHumanoidController);
      momentumBasedController.getYoVariableRegistry().addChild(highLevelControllerRegistry);

      return momentumBasedController;
   }

   public int getSimulationTicksPerControlTick()
   {
      return 50;
   }
}
