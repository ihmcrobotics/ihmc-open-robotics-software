
//A Desired Footstep is always defined in the support foot frame

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SimpleWorldDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   protected final DesiredHeadingControlModule desiredHeadingControlModule;
   protected final CommonWalkingReferenceFrames referenceFrames;

   protected final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   protected final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   protected final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   protected final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   protected final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   protected final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);
   protected final SideDependentList<YoFramePoint> footstepPositions = new SideDependentList<YoFramePoint>();
   protected final SideDependentList<YoFrameOrientation> footstepOrientations = new SideDependentList<YoFrameOrientation>();
   protected final SideDependentList<BipedFootInterface> bipedFeet;



   public SimpleWorldDesiredFootstepCalculator(SideDependentList<BipedFootInterface> bipedFeet, CommonWalkingReferenceFrames referenceFrames,
           DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.referenceFrames = referenceFrames;
      this.bipedFeet = bipedFeet;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values())
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression() + "Footstep";

         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSide);

         YoFramePoint footstepPosition = new YoFramePoint(namePrefix + "Position", worldFrame, registry);
         FramePoint ankle = new FramePoint(footFrame);
         ankle.changeFrame(worldFrame);
         footstepPosition.set(ankle);
         footstepPositions.put(robotSide, footstepPosition);

         YoFrameOrientation footstepOrientation = new YoFrameOrientation(namePrefix + "Orientation", "", worldFrame, registry);
         Transform3D footToWorld = footFrame.getTransformToDesiredFrame(worldFrame);
         footstepOrientation.setRotation(footToWorld);
         footstepOrientations.put(robotSide, footstepOrientation);
      }

      parentRegistry.addChild(registry);
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      Matrix3d footToWorldRotation = new Matrix3d();
      footstepOrientations.get(supportLegSide).getMatrix3d(footToWorldRotation);
      double stanceMinZWithRespectToAnkle = DesiredFootstepCalculatorTools.computeMinZWithRespectToAnkleInWorldFrame(footToWorldRotation, referenceFrames.getFootFrame(supportLegSide),
                                               bipedFeet.get(supportLegSide));
      double maxStanceX = DesiredFootstepCalculatorTools.computeMaxXWithRespectToAnkleInFrame(footToWorldRotation, referenceFrames.getFootFrame(supportLegSide),
                             bipedFeet.get(supportLegSide), desiredHeadingControlModule.getDesiredHeadingFrame());

      // roll and pitch with respect to world, yaw with respect to other foot's yaw
      double swingFootYaw = footstepOrientations.get(supportLegSide).getYaw().getDoubleValue() + stepYaw.getDoubleValue();
      double swingFootPitch = stepPitch.getDoubleValue();
      double swingFootRoll = stepRoll.getDoubleValue();
      footstepOrientations.get(swingLegSide).setYawPitchRoll(swingFootYaw, swingFootPitch, swingFootRoll);
      footstepOrientations.get(swingLegSide).getMatrix3d(footToWorldRotation);
      double swingMinZWithRespectToAnkle = DesiredFootstepCalculatorTools.computeMinZWithRespectToAnkleInWorldFrame(footToWorldRotation, referenceFrames.getFootFrame(swingLegSide),
                                              bipedFeet.get(swingLegSide));
      double maxSwingX = DesiredFootstepCalculatorTools.computeMaxXWithRespectToAnkleInFrame(footToWorldRotation, referenceFrames.getFootFrame(swingLegSide),
                            bipedFeet.get(swingLegSide), desiredHeadingControlModule.getDesiredHeadingFrame());

      FramePoint newFootstepPosition = footstepPositions.get(supportLegSide).getFramePointCopy();
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      FrameVector footstepOffset = new FrameVector(desiredHeadingFrame, stepLength.getDoubleValue() + maxStanceX - maxSwingX,
                                      supportLegSide.negateIfLeftSide(stepWidth.getDoubleValue()),
                                      stepHeight.getDoubleValue() + stanceMinZWithRespectToAnkle - swingMinZWithRespectToAnkle);
      footstepOffset.changeFrame(newFootstepPosition.getReferenceFrame());
      newFootstepPosition.add(footstepOffset);
      footstepPositions.get(swingLegSide).set(newFootstepPosition);
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
}
