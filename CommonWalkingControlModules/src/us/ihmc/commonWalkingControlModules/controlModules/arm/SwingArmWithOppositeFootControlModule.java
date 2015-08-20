package us.ihmc.commonWalkingControlModules.controlModules.arm;

import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class SwingArmWithOppositeFootControlModule extends PDArmControlModule
{
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final SideDependentList<ReferenceFrame> armAttachmentFrames;
   private final ArmJointName[] armJointNames;
   private final double armLength;
   private final double swingMultiplier;

   public SwingArmWithOppositeFootControlModule(ProcessedSensorsInterface processedSensors, CommonHumanoidReferenceFrames referenceFrames, double controlDT, SideDependentList<ReferenceFrame> armAttachmentFrames, ArmJointName[] armJointNames, double armLength, double swingMultiplier, YoVariableRegistry parentRegistry)
   {
      super(processedSensors, controlDT, parentRegistry);
      this.referenceFrames = referenceFrames;
      this.armAttachmentFrames = new SideDependentList<ReferenceFrame>(armAttachmentFrames);
      this.armJointNames = armJointNames;
      this.armLength = armLength;
      this.swingMultiplier = swingMultiplier;
   }

   protected void computeDesireds()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : armJointNames)
         {
            desiredArmJointPositions.get(robotSide).get(armJointName).set(0.0);
            desiredArmJointVelocities.get(robotSide).get(armJointName).set(0.0);
         }
         
         desiredArmJointPositions.get(robotSide).get(ArmJointName.ELBOW_PITCH).set(-0.3); // bend elbow a little
         
         final RobotSide oppositeSide = robotSide.getOppositeSide();
         final ReferenceFrame oppositeFootFrame = referenceFrames.getFootFrame(oppositeSide);
         FramePoint oppositeFootPosition = new FramePoint(oppositeFootFrame);
         oppositeFootPosition.changeFrame(armAttachmentFrames.get(robotSide));
         
         double handX = oppositeFootPosition.getX();
         final double sine = MathTools.clipToMinMax(handX / armLength, -1.0, 1.0);
         double qShoulderPitch = -swingMultiplier * Math.asin(sine);
         desiredArmJointPositions.get(robotSide).get(ArmJointName.SHOULDER_PITCH).set(qShoulderPitch);
      }
   }

   protected void setGains()
   {
      for (RobotSide robotSide : RobotSide.values)
      {         
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setProportionalGain(200.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW_PITCH).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.WRIST_ROLL).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH).setProportionalGain(100.0);

         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setDerivativeGain(2.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW_PITCH).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.WRIST_ROLL).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH).setDerivativeGain(10.0);
      }      
   }
}
