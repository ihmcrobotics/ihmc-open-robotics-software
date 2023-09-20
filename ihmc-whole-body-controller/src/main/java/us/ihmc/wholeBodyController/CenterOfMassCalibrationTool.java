package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpPreserveYReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CenterOfMassCalibrationTool implements Updatable
{
   private static final boolean DEBUG = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   
   private final YoFramePoint3D spinePitchCoMInZUpFrame, leftHipPitchCoMInZUpFrame, rightHipPitchCoMInZUpFrame, leftKneeCoMInZUpFrame, rightKneeCoMInZUpFrame;

   private final CenterOfMassCalculator spinePitchCenterOfMassCalculator, leftHipPitchCenterOfMassCalculator, rightHipPitchCenterOfMassCalculator,
           leftKneeCenterOfMassCalculator, rightKneeCenterOfMassCalculator;
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final YoGraphicCoordinateSystem spinePitchZUpFrameViz;
   private final YoGraphicCoordinateSystem leftHipPitchFrameViz, leftHipPitchZUpFrameViz;

   private final FullHumanoidRobotModel fullRobotModel;
   private final ForceSensorDataHolderReadOnly forceSensorDataHolder;
   private final SideDependentList<ForceSensorDataReadOnly> ankleForceSensors = new SideDependentList<>();
   
   private final YoDouble leftKneeTorqueCheck = new YoDouble("leftKneeTorqueCheck", registry);
   
   public CenterOfMassCalibrationTool(FullHumanoidRobotModel fullRobotModel, ForceSensorDataHolderReadOnly forceSensorDataHolder, YoGraphicsListRegistry yoGraphicsListRegistry,
                                      YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.forceSensorDataHolder = forceSensorDataHolder;
            
      ForceSensorDataReadOnly leftAnkleForceSensor = forceSensorDataHolder.getData("LeftAnkle");
      ForceSensorDataReadOnly rightAnkleForceSensor = forceSensorDataHolder.getData("RightAnkle");
      ankleForceSensors.put(RobotSide.LEFT, leftAnkleForceSensor);
      ankleForceSensors.put(RobotSide.RIGHT, rightAnkleForceSensor);
      
      
      
      RigidBodyBasics spinePitchBody = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH).getSuccessor();
      spinePitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(spinePitchBody, true);
      spinePitchCoMInZUpFrame = new YoFramePoint3D("spinePitchCoMInZUpFrame", spinePitchCenterOfMassCalculator.getReferenceFrame(), registry);

      RigidBodyBasics leftHipPitchBody = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getSuccessor();
      leftHipPitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(leftHipPitchBody, true);
      leftHipPitchCoMInZUpFrame = new YoFramePoint3D("leftHipPitchCoMInZUpFrame", leftHipPitchCenterOfMassCalculator.getReferenceFrame(), registry);

      RigidBodyBasics rightHipPitchBody = fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH).getSuccessor();
      rightHipPitchCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(rightHipPitchBody, true);
      rightHipPitchCoMInZUpFrame = new YoFramePoint3D("rightHipPitchCoMInZUpFrame", rightHipPitchCenterOfMassCalculator.getReferenceFrame(), registry);

      RigidBodyBasics leftKneeBody = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getSuccessor();
      leftKneeCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(leftKneeBody, true);
      leftKneeCoMInZUpFrame = new YoFramePoint3D("leftKneeCoMInZUpFrame", leftKneeCenterOfMassCalculator.getReferenceFrame(), registry);

      RigidBodyBasics rightKneeBody = fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH).getSuccessor();
      rightKneeCenterOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(rightKneeBody, true);
      rightKneeCoMInZUpFrame = new YoFramePoint3D("rightKneeCoMInZUpFrame", rightKneeCenterOfMassCalculator.getReferenceFrame(), registry);

      spinePitchZUpFrameViz = new YoGraphicCoordinateSystem("spinePitchZUpFrameViz", "", registry, true, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("CenterOfMassCalibrationTool", spinePitchZUpFrameViz);
      
      leftHipPitchZUpFrameViz = new YoGraphicCoordinateSystem("leftHipPitchZUpFrameViz", "", registry, true, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("CenterOfMassCalibrationTool", leftHipPitchZUpFrameViz);
      
      leftHipPitchFrameViz = new YoGraphicCoordinateSystem("leftHipPitchFrameViz", "", registry, true, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("CenterOfMassCalibrationTool", leftHipPitchFrameViz);
      
      parentRegistry.addChild(registry);
   }

   @Override
   public void update(double time)
   {
      spinePitchZUpFrameViz.setToReferenceFrame(spinePitchCenterOfMassCalculator.getReferenceFrame());
      leftHipPitchZUpFrameViz.setToReferenceFrame(leftHipPitchCenterOfMassCalculator.getReferenceFrame());
      leftHipPitchFrameViz.setToReferenceFrame(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint());

      spinePitchCenterOfMassCalculator.getReferenceFrame().update();
      leftHipPitchCenterOfMassCalculator.getReferenceFrame().update();
      rightHipPitchCenterOfMassCalculator.getReferenceFrame().update();
      leftKneeCenterOfMassCalculator.getReferenceFrame().update();
      rightKneeCenterOfMassCalculator.getReferenceFrame().update();


      spinePitchCenterOfMassCalculator.reset();
      tempFramePoint.setIncludingFrame(spinePitchCenterOfMassCalculator.getCenterOfMass());
      spinePitchCoMInZUpFrame.set(tempFramePoint);

      leftHipPitchCenterOfMassCalculator.reset();
      tempFramePoint.setIncludingFrame(leftHipPitchCenterOfMassCalculator.getCenterOfMass());
      leftHipPitchCoMInZUpFrame.set(tempFramePoint);

      rightHipPitchCenterOfMassCalculator.reset();
      tempFramePoint.setIncludingFrame(rightHipPitchCenterOfMassCalculator.getCenterOfMass());
      rightHipPitchCoMInZUpFrame.set(tempFramePoint);

      leftKneeCenterOfMassCalculator.reset();
      tempFramePoint.setIncludingFrame(leftKneeCenterOfMassCalculator.getCenterOfMass());
      leftKneeCoMInZUpFrame.set(tempFramePoint);

      rightKneeCenterOfMassCalculator.reset();
      tempFramePoint.setIncludingFrame(rightKneeCenterOfMassCalculator.getCenterOfMass());
      rightKneeCoMInZUpFrame.set(tempFramePoint);
      
      
//      for (RobotSide robotSide : RobotSide.values)
         RobotSide robotSide = RobotSide.LEFT;
         
      {
         ForceSensorDataReadOnly forceSensorData = ankleForceSensors.get(robotSide);
         ReferenceFrame measurementFrame = forceSensorData.getMeasurementFrame();
         FrameVector3D footForce = new FrameVector3D(forceSensorData.getWrench().getLinearPart());
         FrameVector3D footTorque = new FrameVector3D(forceSensorData.getWrench().getAngularPart());
         
         ReferenceFrame jointFrame = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH).getFrameAfterJoint();
         
         FramePoint3D forceSensorLocation = new FramePoint3D(measurementFrame);
         forceSensorLocation.changeFrame(jointFrame);
         footForce.changeFrame(jointFrame);
         
         FrameVector3D cross = new FrameVector3D(jointFrame);
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

   private static CenterOfMassCalculator createCenterOfMassCalculatorInJointZUpFrame(RigidBodyBasics rootBody, boolean preserveY)
   {
      if (DEBUG) System.out.println("\nCenterOfMassCalibrationTool: rootBody = " + rootBody);

      JointBasics parentJoint = rootBody.getParentJoint();
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
         jointZUpFrame = new ZUpFrame(jointFrame, jointName + "ZUp");
      }
      
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, jointZUpFrame);

      return centerOfMassCalculator;
   }
}
