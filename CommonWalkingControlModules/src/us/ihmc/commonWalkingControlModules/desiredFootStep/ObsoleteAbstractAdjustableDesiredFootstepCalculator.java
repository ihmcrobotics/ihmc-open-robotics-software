package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class ObsoleteAbstractAdjustableDesiredFootstepCalculator implements ObsoleteDesiredFootstepCalculator
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final SideDependentList<YoFramePoint> footstepPositions = new SideDependentList<YoFramePoint>();
   protected final SideDependentList<YoFrameOrientation> footstepOrientations = new SideDependentList<YoFrameOrientation>();
   protected final SideDependentList<? extends ContactablePlaneBody> contactableBodies;

   public ObsoleteAbstractAdjustableDesiredFootstepCalculator(SideDependentList<? extends ContactablePlaneBody> contactableBodies,
         SideDependentList<ReferenceFrame> framesToSaveFootstepIn, YoVariableRegistry parentRegistry)
   {
      this.contactableBodies = contactableBodies;

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression() + "Footstep";

         ReferenceFrame frameToSaveFootstepIn = framesToSaveFootstepIn.get(robotSide);
         YoFramePoint footstepPosition = new YoFramePoint(namePrefix + "Position", frameToSaveFootstepIn, registry);
         footstepPositions.put(robotSide, footstepPosition);

         YoFrameOrientation footstepOrientation = new YoFrameOrientation(namePrefix + "Orientation", "", frameToSaveFootstepIn, registry);
         footstepOrientations.put(robotSide, footstepOrientation);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public final Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      FramePose footstepPose = new FramePose(footstepPositions.get(swingLegSide).getFramePointCopy(),
            footstepOrientations.get(swingLegSide).getFrameOrientationCopy());
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", footstepPose);

      ContactablePlaneBody foot = contactableBodies.get(swingLegSide);

      boolean trustHeight = true;
      Footstep desiredFootstep = new Footstep(foot.getRigidBody(), swingLegSide, foot.getSoleFrame(), poseReferenceFrame, trustHeight);

      return desiredFootstep;
   }

   @Override
   public Footstep predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, Footstep desiredFootstep)
   {
      return null;
   }
}
