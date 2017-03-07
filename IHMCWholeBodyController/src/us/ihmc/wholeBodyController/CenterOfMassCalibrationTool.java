package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpPreserveYReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;

public class CenterOfMassCalibrationTool implements Updatable
{
   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final YoFramePoint spinePitchCoMInZUpFrame, leftHipPitchCoMInZUpFrame, rightHipPitchCoMInZUpFrame, leftKneeCoMInZUpFrame, rightKneeCoMInZUpFrame;

   private final CenterOfMassCalculator spinePitchCenterOfMassCalculator, leftHipPitchCenterOfMassCalculator, rightHipPitchCenterOfMassCalculator,
           leftKneeCenterOfMassCalculator, rightKneeCenterOfMassCalculator;
   private final FramePoint tempFramePoint = new FramePoint();
   private final YoGraphicCoordinateSystem spinePitchZUpFrameViz;
   private final YoGraphicCoordinateSystem leftHipPitchFrameViz, leftHipPitchZUpFrameViz;

   private final FullHumanoidRobotModel fullRobotModel;
   private final ForceSensorDataHolderReadOnly forceSensorDataHolder;
   private final SideDependentList<ForceSensorDataReadOnly> ankleForceSensors = new SideDependentList<>();
   
   private final DoubleYoVariable leftKneeTorqueCheck = new DoubleYoVariable("leftKneeTorqueCheck", registry);
   
   public CenterOfMassCalibrationTool(FullHumanoidRobotModel fullRobotModel, ForceSensorDataHolderReadOnly forceSensorDataHolder, YoGraphicsListRegistry yoGraphicsListRegistry,
                                      YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.forceSensorDataHolder = forceSensorDataHolder;
            
      ForceSensorDataReadOnly leftAnkleForceSensor = forceSensorDataHolder.getByName("LeftAnkle");
      ForceSensorDataReadOnly rightAnkleForceSensor = forceSensorDataHolder.getByName("RightAnkle");
      ankleForceSensors.put(RobotSide.LEFT, leftAnkleForceSensor);
      ankleForceSensors.put(RobotSide.RIGHT, rightAnkleForceSensor);
      
      
      
      RigidBody spinePitchBody = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH).getSuccessor();
      spinePitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(spinePitchBody, true);
      spinePitchCoMInZUpFrame = new YoFramePoint("spinePitchCoMInZUpFrame", spinePitchCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody leftHipPitchBody = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getSuccessor();
      leftHipPitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(leftHipPitchBody, true);
      leftHipPitchCoMInZUpFrame = new YoFramePoint("leftHipPitchCoMInZUpFrame", leftHipPitchCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody rightHipPitchBody = fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH).getSuccessor();
      rightHipPitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(rightHipPitchBody, true);
      rightHipPitchCoMInZUpFrame = new YoFramePoint("rightHipPitchCoMInZUpFrame", rightHipPitchCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody leftKneeBody = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getSuccessor();
      leftKneeCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(leftKneeBody, true);
      leftKneeCoMInZUpFrame = new YoFramePoint("leftKneeCoMInZUpFrame", leftKneeCenterOfMassCalculator.getDesiredFrame(), registry);

      RigidBody rightKneeBody = fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH).getSuccessor();
      rightKneeCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(rightKneeBody, true);
      rightKneeCoMInZUpFrame = new YoFramePoint("rightKneeCoMInZUpFrame", rightKneeCenterOfMassCalculator.getDesiredFrame(), registry);

      spinePitchZUpFrameViz = new YoGraphicCoordinateSystem("spinePitchZUpFrameViz", "", registry, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("CenterOfMassCalibrationTool", spinePitchZUpFrameViz);
      
      leftHipPitchZUpFrameViz = new YoGraphicCoordinateSystem("leftHipPitchZUpFrameViz", "", registry, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("CenterOfMassCalibrationTool", leftHipPitchZUpFrameViz);
      
      leftHipPitchFrameViz = new YoGraphicCoordinateSystem("leftHipPitchFrameViz", "", registry, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("CenterOfMassCalibrationTool", leftHipPitchFrameViz);
      
      parentRegistry.addChild(registry);
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
      
      
//      for (RobotSide robotSide : RobotSide.values)
         RobotSide robotSide = RobotSide.LEFT;
         
      {
         Wrench footWrench = new Wrench();
         ForceSensorDataReadOnly forceSensorData = ankleForceSensors.get(robotSide);
         ReferenceFrame measurementFrame = forceSensorData.getMeasurementFrame();
         forceSensorData.getWrench(footWrench);
         FrameVector footForce = footWrench.getLinearPartAsFrameVectorCopy();
         FrameVector footTorque = footWrench.getAngularPartAsFrameVectorCopy();
         
         ReferenceFrame jointFrame = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH).getFrameAfterJoint();
         
         FramePoint forceSensorLocation = new FramePoint(measurementFrame);
         forceSensorLocation.changeFrame(jointFrame);
         footForce.changeFrame(jointFrame);
         
         FrameVector cross = new FrameVector(jointFrame);
         cross.cross(forceSensorLocation, footForce);
         
         footTorque.changeFrame(jointFrame);
         cross.add(footTorque);
         
         leftKneeTorqueCheck.set(cross.getY());

//         ForceSensorData footForceSensor = forceSensorDataHolder.getByName(footSensorNames.get(robotSide));
//         WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(bipedFeet.get(robotSide).getName(), footForceSensor, 0.02, totalRobotWeight,
//               bipedFeet.get(robotSide), YoGraphicsListRegistry, contactThresholdForce, registry);
      }
      
   
   }

   
//   private void readSensorData(Wrench footWrench)
//   {
//      forceSensorData.packWrench(footWrench);
//
//      footForce.setToZero(footWrench.getExpressedInFrame());
//      footWrench.packLinearPart(footForce);
//      yoFootForce.set(footForce);
//      footForce.changeFrame(contactablePlaneBody.getFrameAfterParentJoint());
//      yoFootForceInFoot.set(footForce);
//      footForceMagnitude.set(footForce.length());
//
//      // magnitude of force part is independent of frame
//      footForceMagnitude.set(footForce.length());
//
//      footTorque.setToZero(footWrench.getExpressedInFrame());
//      footWrench.packAngularPart(footTorque);
//      yoFootTorque.set(footTorque);
//      footTorque.changeFrame(contactablePlaneBody.getFrameAfterParentJoint());
//      yoFootTorqueInFoot.set(footTorque);
//
//      updateSensorVisualizer();
//
//   }

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
