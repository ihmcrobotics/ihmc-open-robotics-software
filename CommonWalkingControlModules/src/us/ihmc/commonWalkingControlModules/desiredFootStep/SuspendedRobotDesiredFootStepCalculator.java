package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;


public class SuspendedRobotDesiredFootStepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final DoubleYoVariable currentStepLength = new DoubleYoVariable("currentStepLength", registry);
   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);

   private final IntegerYoVariable state = new IntegerYoVariable("state", registry);
   private final SideDependentList<? extends ContactablePlaneBody> contactableBodies;

   public SuspendedRobotDesiredFootStepCalculator(SideDependentList<? extends ContactablePlaneBody> contactableBodies,
         SideDependentList<ReferenceFrame> ankleZUpFrames, DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      this.contactableBodies = contactableBodies;
      this.ankleZUpFrames = ankleZUpFrames;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      parentRegistry.addChild(registry);

      stepLength.set(0.20);
      stepWidth.set(0.22);
      stepHeight.set(0.0);
      stepYaw.set(0.0);
      stepPitch.set(0.0);
      stepRoll.set(0.0);

      state.set(0);
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      switch (state.getIntegerValue())
      {
      case 0:
         currentStepLength.set(stepLength.getDoubleValue());

         break;

      case 1:
         currentStepLength.set(-stepLength.getDoubleValue());

         break;
      }

      state.increment();
      if (state.getIntegerValue() > 1)
         state.set(0);

      updateAndGetDesiredFootstep(supportLegSide);
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      // Footstep Frame
      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportLegSide);
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      // Footstep Position
      FramePoint footstepPosition = new FramePoint(supportAnkleZUpFrame);
      FrameVector footstepOffset = new FrameVector(desiredHeadingFrame, currentStepLength.getDoubleValue(), supportLegSide.negateIfLeftSide(stepWidth
            .getDoubleValue()), stepHeight.getDoubleValue());

      footstepPosition.changeFrame(desiredHeadingFrame);

      footstepPosition.add(footstepOffset);

      // Footstep Orientation
      FrameOrientation footstepOrientation = new FrameOrientation(desiredHeadingFrame);
      footstepOrientation.setYawPitchRoll(stepYaw.getDoubleValue(), stepPitch.getDoubleValue(), stepRoll.getDoubleValue());

      // Create a foot Step Pose from Position and Orientation
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", footstepPose);

      ContactablePlaneBody contactableBody = contactableBodies.get(swingLegSide);

      boolean trustHeight = false;
      Footstep desiredFootstep = new Footstep(contactableBody.getRigidBody(), swingLegSide, contactableBody.getSoleFrame(), poseReferenceFrame, trustHeight);

      return desiredFootstep;
   }

   public void setupParametersForM2V2()
   {
      // TODO Auto-generated method stub
   }

   public void setupParametersForR2()
   {
      // TODO Auto-generated method stub
   }

   public Footstep predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, Footstep desiredFootstep)
   {
      return null;
   }

   public boolean isDone()
   {
      return false;
   }

   @Override
   public void initialize()
   {
   }
}