package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PushRecoveryDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PushRecoveryDesiredStepLocationCalcualtor");

   private Footstep desiredFootstep;

   private final CommonWalkingReferenceFrames referenceFrames;
   private final CouplingRegistry couplingRegistry;

   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);

   private final DoubleYoVariable percentTowardCentroid = new DoubleYoVariable("percentTowardCentroid",
                                                             "Percent to step from Instantaneous Capture Point to Capture Region Centroid", registry);


   // Constructor
   public PushRecoveryDesiredFootstepCalculator(CommonWalkingReferenceFrames referenceFrames, CouplingRegistry couplingRegistry,
           YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      this.referenceFrames = referenceFrames;
      this.couplingRegistry = couplingRegistry;

      percentTowardCentroid.set(0.75);
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
      ReferenceFrame supportFootFrame = referenceFrames.getAnkleZUpFrame(supportLegSide);
      FrameConvexPolygon2d captureRegion = couplingRegistry.getCaptureRegion();

      if (captureRegion == null)
      {
         computePushRecoveryHomePositionFootstep(supportLegSide, supportFootFrame);
      }
      else
      {
         FramePoint capturePoint = couplingRegistry.getCapturePointInFrame(captureRegion.getReferenceFrame());
         FramePoint2d capturePoint2d = capturePoint.toFramePoint2d();

//       capturePoint2d = capturePoint2d.changeFrameCopy(captureRegion.getReferenceFrame());

         FramePoint2d captureRegionCentroid = captureRegion.getCentroidCopy();
         captureRegionCentroid = FramePoint2d.morph(capturePoint2d, captureRegionCentroid, percentTowardCentroid.getDoubleValue());
         captureRegionCentroid = captureRegionCentroid.changeFrameCopy(supportFootFrame);

         // Footstep Position
         stepLength.set(captureRegionCentroid.getX());
         stepWidth.set(captureRegionCentroid.getY());
         stepHeight.set(0.0);

         FramePoint footstepPosition = new FramePoint(captureRegionCentroid.getReferenceFrame(), stepLength.getDoubleValue(), stepWidth.getDoubleValue(),
                                          stepHeight.getDoubleValue());

         // Footstep Orientation
         Orientation footstepOrientation = new Orientation(footstepPosition.getReferenceFrame());
         stepYaw.set(0.0);
         stepPitch.set(0.0);
         stepRoll.set(0.0);
         footstepOrientation.setYawPitchRoll(stepYaw.getDoubleValue(), stepPitch.getDoubleValue(), stepRoll.getDoubleValue());

         // Create a foot Step Pose from Position and Orientation
         FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
         desiredFootstep = new Footstep(supportLegSide.getOppositeSide(), footstepPose);
      }
      
      return desiredFootstep;
   }

   private void computePushRecoveryHomePositionFootstep(RobotSide supportLegSide, ReferenceFrame supportFootFrame)
   {
      // Footstep Position
      stepLength.set(0.0);
      stepWidth.set(supportLegSide.negateIfLeftSide(0.3));
      stepHeight.set(0.15);
      FramePoint footstepPosition = new FramePoint(supportFootFrame, stepLength.getDoubleValue(), stepWidth.getDoubleValue(), stepHeight.getDoubleValue());

      // Foot Step Orientation
      Orientation footstepOrientation = new Orientation(footstepPosition.getReferenceFrame());
      footstepOrientation.setYawPitchRoll(0.0, 0.0, 0.0);

      // Create a foot Step Pose from Position and Orientation
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      desiredFootstep = new Footstep(supportLegSide, footstepPose);
   }

   public void setupParametersForM2V2()
   {      
   }

   public void setupParametersForR2()
   {      
   }
}
