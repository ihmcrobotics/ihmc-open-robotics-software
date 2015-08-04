
//A Desired Footstep is always defined in the support foot frame

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class SimpleWorldDesiredFootstepCalculator extends AbstractAdjustableDesiredFootstepCalculator
{
   protected final DesiredHeadingControlModule desiredHeadingControlModule;
   protected final CommonHumanoidReferenceFrames referenceFrames;

   protected final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   protected final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   protected final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   protected final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   protected final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   protected final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);
   protected final SideDependentList<ContactablePlaneBody> bipedFeet;

   public SimpleWorldDesiredFootstepCalculator(SideDependentList<ContactablePlaneBody> bipedFeet, CommonHumanoidReferenceFrames referenceFrames,
           DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      super(bipedFeet, getFramesToStoreFootstepsIn(), parentRegistry);
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.referenceFrames = referenceFrames;
      this.bipedFeet = bipedFeet;

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);

         FramePose currentFootPose = new FramePose(footFrame);

         FramePoint initialFootPosition = new FramePoint();
         currentFootPose.getPositionIncludingFrame(initialFootPosition);
         YoFramePoint footstepPosition = footstepPositions.get(robotSide);
         initialFootPosition.changeFrame(footstepPosition.getReferenceFrame());
         footstepPosition.set(initialFootPosition);

         FrameOrientation initialFootOrientation = new FrameOrientation();
         currentFootPose.getOrientationIncludingFrame(initialFootOrientation);
         YoFrameOrientation footstepOrientation = footstepOrientations.get(robotSide);
         initialFootOrientation.changeFrame(footstepOrientation.getReferenceFrame());
         footstepOrientation.set(initialFootOrientation);
      }
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      Matrix3d footToWorldRotation = new Matrix3d();
      footstepOrientations.get(supportLegSide).getMatrix3d(footToWorldRotation);
      double stanceMinZWithRespectToAnkle = DesiredFootstepCalculatorTools.computeMinZPointWithRespectToAnkleInWorldFrame(footToWorldRotation,
                                               bipedFeet.get(supportLegSide));
      double maxStanceX = DesiredFootstepCalculatorTools.computeMaxXWithRespectToAnkleInFrame(footToWorldRotation, bipedFeet.get(supportLegSide),
                             desiredHeadingControlModule.getDesiredHeadingFrame());

      // roll and pitch with respect to world, yaw with respect to other foot's yaw
      double swingFootYaw = footstepOrientations.get(supportLegSide).getYaw().getDoubleValue() + stepYaw.getDoubleValue();
      double swingFootPitch = stepPitch.getDoubleValue();
      double swingFootRoll = stepRoll.getDoubleValue();
      footstepOrientations.get(swingLegSide).setYawPitchRoll(swingFootYaw, swingFootPitch, swingFootRoll);
      footstepOrientations.get(swingLegSide).getMatrix3d(footToWorldRotation);
      double swingMinZWithRespectToAnkle = DesiredFootstepCalculatorTools.computeMinZPointWithRespectToAnkleInWorldFrame(footToWorldRotation,
                                              bipedFeet.get(swingLegSide));
      double maxSwingX = DesiredFootstepCalculatorTools.computeMaxXWithRespectToAnkleInFrame(footToWorldRotation, bipedFeet.get(swingLegSide),
                            desiredHeadingControlModule.getDesiredHeadingFrame());

      FramePoint newFootstepPosition = footstepPositions.get(supportLegSide).getFramePointCopy();
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      FrameVector footstepOffset = new FrameVector(desiredHeadingFrame, stepLength.getDoubleValue() + maxStanceX - maxSwingX,
                                      supportLegSide.negateIfLeftSide(stepWidth.getDoubleValue()),
                                      stepHeight.getDoubleValue() + stanceMinZWithRespectToAnkle - swingMinZWithRespectToAnkle);
      footstepOffset.changeFrame(newFootstepPosition.getReferenceFrame());
      newFootstepPosition.add(footstepOffset);
      footstepPositions.get(swingLegSide).set(newFootstepPosition);
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
      stepHeight.set(0.35);    // 0.25);
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

   private static SideDependentList<ReferenceFrame> getFramesToStoreFootstepsIn()
   {
      return new SideDependentList<ReferenceFrame>(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   protected List<FramePoint> getContactPoints(RobotSide swingSide)
   {
      return contactableBodies.get(swingSide).getContactPointsCopy();
   }
   
   public boolean isDone()
   {
      return false;
   }
}
