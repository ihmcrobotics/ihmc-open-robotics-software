package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SquareUpStepCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorld;
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private final FramePoint2D leftToRightFootDistance = new FramePoint2D();

   private final FramePoint3D squareUpPosition = new FramePoint3D();
   private final FrameVector2D squaringStepDirection = new FrameVector2D();


   private final FramePoint2DBasics squareUpLocation = new FramePoint2D();
   private final FramePose3D stancePose = new FramePose3D();

   private final DoubleProvider squareUpPreferredStanceWidth;
   private final DoubleProvider maxAllowedFinalStepXOffset;
   private final FootstepTiming squareUpStepTiming = new FootstepTiming();

   public SquareUpStepCalculator(SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorld,
                                 SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                 PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                 YoRegistry registry)
   {
      this.footPolygonsInWorld = footPolygonsInWorld;
      this.soleZUpFrames = soleZUpFrames;

      squareUpPreferredStanceWidth = new DoubleParameter("squareUpPreferredStanceWidth", registry, pushRecoveryControllerParameters.getPreferredStepWidth());
      squareUpStepTiming.setTimings(pushRecoveryControllerParameters.getMinimumRecoverySwingDuration(), pushRecoveryControllerParameters.getRecoveryTransferDuration());
      maxAllowedFinalStepXOffset = new DoubleParameter("maxAllowedFinalStepXOffset", registry, pushRecoveryControllerParameters.getMaxAllowedFinalStepXOffset());
   }

   public RobotSide computeSideOnCapturePoint(FramePoint2DReadOnly capturePoint2d)
   {
      // first check if feet are already close to the preferred standing pose
      if(isRobotStanceCloseToPreferred())
         return null;

      if(footPolygonsInWorld.get(RobotSide.LEFT).isPointInside(capturePoint2d))
         return RobotSide.LEFT;
      else if(footPolygonsInWorld.get(RobotSide.RIGHT).isPointInside(capturePoint2d))
         return RobotSide.RIGHT;
      return null;
   }

   private boolean isRobotStanceCloseToPreferred()
   {
      FramePoint2DReadOnly leftFootCentroid = footPolygonsInWorld.get(RobotSide.LEFT).getCentroid();
      FramePoint2DReadOnly rightFootCentroid = footPolygonsInWorld.get(RobotSide.RIGHT).getCentroid();

      leftToRightFootDistance.setToZero();
      double xDistance = leftFootCentroid.getX() - rightFootCentroid.getX();
      double yDistance = leftFootCentroid.getY() - rightFootCentroid.getY();
      leftToRightFootDistance.set(xDistance, yDistance);
      leftToRightFootDistance.changeFrame(soleZUpFrames.get(RobotSide.RIGHT));
      return Math.abs(leftToRightFootDistance.getX()) < maxAllowedFinalStepXOffset.getValue();
   }


   public void computeSquareUpStep(RobotSide nextSupportSide, Footstep squareUpStepToPack)
   {
      squareUpLocation.setToZero(soleZUpFrames.get(nextSupportSide));
      squareUpLocation.changeFrame(worldFrame);

      stancePose.setToZero(soleZUpFrames.get(nextSupportSide));
      stancePose.changeFrame(worldFrame);

      // set square position at preferred width distance from next support foot
      squaringStepDirection.setToZero(soleZUpFrames.get(nextSupportSide));
      squaringStepDirection.add(0.0, nextSupportSide.negateIfLeftSide(squareUpPreferredStanceWidth.getValue()));
      squaringStepDirection.changeFrame(worldFrame);
      squareUpLocation.add(squaringStepDirection);
      squareUpPosition.set(squareUpLocation);

      squareUpStepToPack.setPose(squareUpPosition, stancePose.getOrientation());
      squareUpStepToPack.setRobotSide(nextSupportSide.getOppositeSide());
   }

   public FootstepTiming getSquareUpStepTiming()
   {
      return squareUpStepTiming;
   }
}
