package us.ihmc.darpaRoboticsChallenge.calib;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.ZUpFrame;
import us.ihmc.utilities.math.geometry.ZUpPreserveYReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class CenterOfMassCalibrationTool implements Updatable
{
   private static final boolean DEBUG = false;

   private final YoFramePoint spinePitchCoMInZUpFrame, leftHipPitchCoMInZUpFrame, rightHipPitchCoMInZUpFrame, leftKneeCoMInZUpFrame, rightKneeCoMInZUpFrame;

   private final CenterOfMassCalculator spinePitchCenterOfMassCalculator, leftHipPitchCenterOfMassCalculator, rightHipPitchCenterOfMassCalculator,
           leftKneeCenterOfMassCalculator, rightKneeCenterOfMassCalculator;
   private final FramePoint tempFramePoint = new FramePoint();
   private final YoGraphicCoordinateSystem spinePitchZUpFrameViz;
   private final YoGraphicCoordinateSystem leftHipPitchFrameViz, leftHipPitchZUpFrameViz;

   private final FullRobotModel fullRobotModel;
   
   public CenterOfMassCalibrationTool(FullRobotModel fullRobotModel, YoGraphicsListRegistry dynamicGraphicObjectsListRegistry,
                                      YoVariableRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;
      
      RigidBody spinePitchBody = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH).getSuccessor();
      spinePitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(spinePitchBody, true);
      spinePitchCoMInZUpFrame = new YoFramePoint("spinePitchCoMInZUpFrame", spinePitchCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody leftHipPitchBody = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getSuccessor();
      leftHipPitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(leftHipPitchBody, true);
      leftHipPitchCoMInZUpFrame = new YoFramePoint("leftHipPitchCoMInZUpFrame", leftHipPitchCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody rightHipPitchBody = fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH).getSuccessor();
      rightHipPitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(rightHipPitchBody, true);
      rightHipPitchCoMInZUpFrame = new YoFramePoint("rightHipPitchCoMInZUpFrame", rightHipPitchCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody leftKneeBody = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE).getSuccessor();
      leftKneeCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(leftKneeBody, true);
      leftKneeCoMInZUpFrame = new YoFramePoint("leftKneeCoMInZUpFrame", leftKneeCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody rightKneeBody = fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE).getSuccessor();
      rightKneeCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(rightKneeBody, true);
      rightKneeCoMInZUpFrame = new YoFramePoint("rightKneeCoMInZUpFrame", rightKneeCenterOfMassCalculator.getDesiredFrame(), registry);

      spinePitchZUpFrameViz = new YoGraphicCoordinateSystem("spinePitchZUpFrameViz", "", registry, 0.3);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CenterOfMassCalibrationTool", spinePitchZUpFrameViz);
      
      leftHipPitchZUpFrameViz = new YoGraphicCoordinateSystem("leftHipPitchZUpFrameViz", "", registry, 0.3);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CenterOfMassCalibrationTool", leftHipPitchZUpFrameViz);
      
      leftHipPitchFrameViz = new YoGraphicCoordinateSystem("leftHipPitchFrameViz", "", registry, 0.3);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CenterOfMassCalibrationTool", leftHipPitchFrameViz);
   }

   @Override
   public void update(double time)
   {
      spinePitchZUpFrameViz.setToReferenceFrame(spinePitchCenterOfMassCalculator.getDesiredFrame());
      leftHipPitchZUpFrameViz.setToReferenceFrame(leftHipPitchCenterOfMassCalculator.getDesiredFrame());
      leftHipPitchFrameViz.setToReferenceFrame(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint());

      spinePitchCenterOfMassCalculator.getDesiredFrame().update();
      leftHipPitchCenterOfMassCalculator.getDesiredFrame().update();
      rightHipPitchCenterOfMassCalculator.getDesiredFrame().update();
      leftKneeCenterOfMassCalculator.getDesiredFrame().update();
      rightKneeCenterOfMassCalculator.getDesiredFrame().update();


      spinePitchCenterOfMassCalculator.compute();
      spinePitchCenterOfMassCalculator.getCenterOfMass(tempFramePoint);
      spinePitchCoMInZUpFrame.set(tempFramePoint);

      leftHipPitchCenterOfMassCalculator.compute();
      leftHipPitchCenterOfMassCalculator.getCenterOfMass(tempFramePoint);
      leftHipPitchCoMInZUpFrame.set(tempFramePoint);

      rightHipPitchCenterOfMassCalculator.compute();
      rightHipPitchCenterOfMassCalculator.getCenterOfMass(tempFramePoint);
      rightHipPitchCoMInZUpFrame.set(tempFramePoint);

      leftKneeCenterOfMassCalculator.compute();
      leftKneeCenterOfMassCalculator.getCenterOfMass(tempFramePoint);
      leftKneeCoMInZUpFrame.set(tempFramePoint);

      rightKneeCenterOfMassCalculator.compute();
      rightKneeCenterOfMassCalculator.getCenterOfMass(tempFramePoint);
      rightKneeCoMInZUpFrame.set(tempFramePoint);
   }


   private static CenterOfMassCalculator createCenterOfMassCalculatorInJointZUpFrame(RigidBody rootBody, boolean preserveY)
   {
      if (DEBUG) System.out.println("\nCenterOfMassCalibrationTool: rootBody = " + rootBody);

      InverseDynamicsJoint parentJoint = rootBody.getParentJoint();
      if (DEBUG) System.out.println("parentJoint = " + parentJoint);

      ReferenceFrame jointFrame = parentJoint.getFrameAfterJoint();
      if (DEBUG) System.out.println("jointFrame = " + jointFrame);

      String jointName = parentJoint.getName();
      if (DEBUG) System.out.println("jointName = " + jointName);

      ReferenceFrame jointZUpFrame;
      
      if (preserveY)
      {
         jointZUpFrame = new ZUpPreserveYReferenceFrame(ReferenceFrame.getWorldFrame(), jointFrame, jointName + "ZUp");
      }
      else
      {
         jointZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), jointFrame, jointName + "ZUp");
      }
      
      RigidBody[] rigidBodies = ScrewTools.computeSubtreeSuccessors(rootBody.getParentJoint());
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rigidBodies, jointZUpFrame);

      return centerOfMassCalculator;
   }
}
