
//A Desired Footstep is always defined in the support foot frame

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SimpleDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleFootstepCalculator");

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   private Footstep desiredFootstep;

   private final CommonWalkingReferenceFrames referenceFrames;

   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);

   // Constructor
   public SimpleDesiredFootstepCalculator(CommonWalkingReferenceFrames referenceFrames, DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      parentRegistry.addChild(registry);
   }

//   // Getters
//   public Footstep getDesiredFootstep()
//   {
//      return desiredFootstep;
//   }

   // Methods
   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      updateAndGetDesiredFootstep(supportLegSide);
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      // Footstep Frame
      ReferenceFrame supportFootFrame = referenceFrames.getAnkleZUpFrame(supportLegSide);
      ReferenceFrame headingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      // Footstep Position
      FramePoint footstepPosition = new FramePoint(supportFootFrame);
      FrameVector footstepOffset = new FrameVector(headingFrame, stepLength.getDoubleValue(), supportLegSide.negateIfLeftSide(stepWidth.getDoubleValue()), stepHeight.getDoubleValue());
      
      footstepOffset.changeFrame(supportFootFrame);
      footstepPosition.add(footstepOffset); 

      // Footstep Orientation
      Orientation footstepOrientation = new Orientation(headingFrame); 
      updateDesiredFootStepYaw(headingFrame);
      footstepOrientation.setYawPitchRoll(stepYaw.getDoubleValue(), stepPitch.getDoubleValue(), stepRoll.getDoubleValue());
      footstepOrientation.changeFrame(supportFootFrame);
      
      // Create a foot Step Pose from Position and Orientation
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      desiredFootstep = new Footstep(supportLegSide, footstepPose);
      
      return desiredFootstep;
   }

   private void updateDesiredFootStepYaw(ReferenceFrame headingFrame)
   {
//      // Compute the difference between the body and the footstep frames
//      Transform3D headingToFootstepPositionTransform = referenceFrames.getPelvisFrame().getTransformToDesiredFrame(footstepFrame);
//      Matrix3d headingToSwingRotation = new Matrix3d();
//      headingToFootstepPositionTransform.get(headingToSwingRotation);
//      stepYaw.set(RotationFunctions.getYaw(headingToSwingRotation));
   }

   public void setupParametersForM2V2()
   {
      stepLength.set(0.32);
      stepWidth.set(0.2);
      stepHeight.set(0.0);
      stepYaw.set(0.0);
      stepPitch.set(-0.25);
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
}
