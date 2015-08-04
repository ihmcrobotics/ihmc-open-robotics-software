//A Desired Footstep is always defined in the support foot frame
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
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;


public class SimpleDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleFootstepCalculator");

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);
   private final SideDependentList<? extends ContactablePlaneBody> contactableBodies;

   public SimpleDesiredFootstepCalculator(SideDependentList<? extends ContactablePlaneBody> contactableBodies,
         SideDependentList<ReferenceFrame> ankleZUpFrames, DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      this.contactableBodies = contactableBodies;
      this.ankleZUpFrames = ankleZUpFrames;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      parentRegistry.addChild(registry);
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
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
      FrameVector footstepOffset = new FrameVector(desiredHeadingFrame, stepLength.getDoubleValue(),
            supportLegSide.negateIfLeftSide(stepWidth.getDoubleValue()), stepHeight.getDoubleValue());

      footstepPosition.changeFrame(desiredHeadingFrame);

      footstepPosition.add(footstepOffset);

      // Footstep Orientation
      FrameOrientation footstepOrientation = new FrameOrientation(desiredHeadingFrame);
      footstepOrientation.setYawPitchRoll(stepYaw.getDoubleValue(), stepPitch.getDoubleValue(), stepRoll.getDoubleValue());

      // Create a foot Step Pose from Position and Orientation
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);

      ContactablePlaneBody foot = contactableBodies.get(swingLegSide);

      boolean trustHeight = false;
      Footstep desiredFootstep = new Footstep(foot.getRigidBody(), swingLegSide, foot.getSoleFrame(), footstepPoseFrame, trustHeight);

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
      //    stepLength.set(0.21);
      stepWidth.set(0.2);
      stepHeight.set(0.25);
      stepYaw.set(0.0);
      stepPitch.set(0.0);
      stepRoll.set(0.0);
      
      // stairs:
      stepLength.set(0.315);

      // flat ground
      //    stepLength.set(0.3);
      //    stepHeight.set(0.0);
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