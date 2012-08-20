
//A Desired Footstep is always defined in the support foot frame

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SimpleWorldDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleFootstepCalculator");

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);
   private final SideDependentList<YoFramePoint> footstepPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameOrientation> footstepOrientations = new SideDependentList<YoFrameOrientation>();
   private final EnumYoVariable<RobotSide> previousStepSide = EnumYoVariable.create("previousStepSide", RobotSide.class, registry);
   private final SideDependentList<BipedFootInterface> bipedFeet;



   public SimpleWorldDesiredFootstepCalculator(SideDependentList<BipedFootInterface> bipedFeet, CommonWalkingReferenceFrames referenceFrames,
           DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.referenceFrames = referenceFrames;
      this.bipedFeet = bipedFeet;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values())
      {
         String namePrefix = "previous" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Footstep";

         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);

         YoFramePoint previousFootstepPosition = new YoFramePoint(namePrefix + "Position", worldFrame, registry);
         FramePoint ankle = new FramePoint(footFrame);
         ankle.changeFrame(worldFrame);
         previousFootstepPosition.set(ankle);
         footstepPositions.put(robotSide, previousFootstepPosition);

         YoFrameOrientation footstepOrientation = new YoFrameOrientation(namePrefix + "Orientation", "", worldFrame, registry);
         Transform3D footToWorld = footFrame.getTransformToDesiredFrame(worldFrame);
         footstepOrientation.setRotation(footToWorld);
         footstepOrientations.put(robotSide, footstepOrientation);
      }

      previousStepSide.set(null);
      parentRegistry.addChild(registry);
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      Matrix3d footToWorldRotation = new Matrix3d();
      footstepOrientations.get(supportLegSide).getMatrix3d(footToWorldRotation);
      double stanceMinZWithRespectToAnkle = findMinZ(footToWorldRotation, supportLegSide);
      double maxStanceX = findMaxXInDesiredHeadingFrame(footToWorldRotation, supportLegSide);

      // roll and pitch with respect to world, yaw with respect to other foot's yaw
      double swingFootYaw = footstepOrientations.get(supportLegSide).getYaw().getDoubleValue() + stepYaw.getDoubleValue();
      double swingFootPitch = stepPitch.getDoubleValue();
      double swingFootRoll = stepRoll.getDoubleValue();
      footstepOrientations.get(swingLegSide).setYawPitchRoll(swingFootYaw, swingFootPitch, swingFootRoll);
      footstepOrientations.get(swingLegSide).getMatrix3d(footToWorldRotation);
      double swingMinZWithRespectToAnkle = findMinZ(footToWorldRotation, swingLegSide);
      double maxSwingX = findMaxXInDesiredHeadingFrame(footToWorldRotation, swingLegSide);
      
      FramePoint newFootstepPosition = footstepPositions.get(supportLegSide).getFramePointCopy();
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      FrameVector footstepOffset = new FrameVector(desiredHeadingFrame, stepLength.getDoubleValue() + maxStanceX - maxSwingX,
                                      supportLegSide.negateIfLeftSide(stepWidth.getDoubleValue()),
                                      stepHeight.getDoubleValue() + stanceMinZWithRespectToAnkle - swingMinZWithRespectToAnkle);
      footstepOffset.changeFrame(newFootstepPosition.getReferenceFrame());
      newFootstepPosition.add(footstepOffset);
      footstepPositions.get(swingLegSide).set(newFootstepPosition);
   }

   private double findMinZ(Matrix3d footToWorldRotation, RobotSide robotSide)
   {
      ArrayList<FramePoint2d> footPoints = bipedFeet.get(robotSide).getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      double minZ = Double.POSITIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempVector = new Vector3d();
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(referenceFrames.getFootFrame(robotSide));
         tempVector.set(tempFramePoint.getPoint());
         footToWorldRotation.transform(tempVector);
         if (tempVector.getZ() < minZ)
            minZ = tempVector.getZ();
      }

      return minZ;
   }

   private double findMaxXInDesiredHeadingFrame(Matrix3d footToWorldRotation, RobotSide robotSide)
   {
      Transform3D worldToDesiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      ArrayList<FramePoint2d> footPoints = bipedFeet.get(robotSide).getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      double maxX = Double.NEGATIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      Vector3d tempVector = new Vector3d();
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(referenceFrames.getFootFrame(robotSide));
         tempVector.set(tempFramePoint.getPoint()); // foot point w.r.t. ankle in foot frame
         footToWorldRotation.transform(tempVector); // foot point w.r.t. ankle in world frame
         worldToDesiredHeadingFrame.transform(tempVector); // foot point w.r.t. ankle in desired heading frame
         if (tempVector.getX() > maxX)
            maxX = tempVector.getX();
      }

      return maxX;
   }
   
   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      FramePose footstepPose = new FramePose(footstepPositions.get(swingLegSide).getFramePointCopy(),
                                  footstepOrientations.get(swingLegSide).getFrameOrientationCopy());
      Footstep desiredFootstep = new Footstep(swingLegSide, footstepPose);

      return desiredFootstep;
   }

   public void setupParametersForM2V2()
   {
      stepLength.set(0.32);
      stepWidth.set(0.22);
      stepHeight.set(0.0);
      stepYaw.set(0.0);
      stepPitch.set(-0.35);
      stepRoll.set(0.0);
   }

   public void setupParametersForR2()
   {
      stepLength.set(0.6);
      stepWidth.set(0.35);
      stepHeight.set(0.0);
      stepYaw.set(0.0);
      stepPitch.set(-0.25);
      stepRoll.set(0.0);
   }

   public void setupParametersForR2InverseDynamics()
   {
      // stairs:
      stepLength.set(0.15);    // 0.2
      stepWidth.set(0.2);
      stepHeight.set(0.35); // 0.25);
      stepYaw.set(0.0);
      stepPitch.set(0.2);
      stepRoll.set(0.0);

      // flat ground
//    stepLength.set(0.35);
//    stepWidth.set(0.2);
//    stepHeight.set(0.0);
//    stepYaw.set(0.0);
//    stepPitch.set(0.2);
//    stepRoll.set(0.0);
   }
}
