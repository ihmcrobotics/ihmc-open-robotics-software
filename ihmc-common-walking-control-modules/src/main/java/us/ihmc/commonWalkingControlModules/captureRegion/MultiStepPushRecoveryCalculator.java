package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class MultiStepPushRecoveryCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private final ReachableFootholdsCalculator reachableFootholdsCalculator;
   private final AchievableCaptureRegionCalculatorWithDelay captureRegionCalculator;

   private final ConvexPolygonTools polygonTools = new ConvexPolygonTools();

   private final FrameConvexPolygon2D reachableRegion = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();

   private final RecyclingArrayList<FramePoint2DBasics> capturePointsAtTouchdown = new RecyclingArrayList<>(FramePoint2D::new);
   private final RecyclingArrayList<FramePoint2DBasics> recoveryStepLocations = new RecyclingArrayList<>(FramePoint2D::new);

   private final RecyclingArrayList<FrameConvexPolygon2DBasics> captureRegionsAtTouchdown = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2DBasics> reachableRegions = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2DBasics> intersectingRegions = new RecyclingArrayList<>(FrameConvexPolygon2D::new);

   private final PoseReferenceFrame stanceFrame = new PoseReferenceFrame("StanceFrame", worldFrame);
   private final FramePose3D stancePose = new FramePose3D();
   private final FramePoint2D stancePosition = new FramePoint2D();

   private final FrameLine2D forwardLineAtNominalWidth = new FrameLine2D();
   private final FramePoint2D forwardLineSegmentEnd = new FramePoint2D();

   private final FramePoint3D stepPosition = new FramePoint3D();
   private final FramePoint3D squareUpPosition = new FramePoint3D();
   private final FrameVector2D squaringStepDirection = new FrameVector2D();

   private final ConvexPolygon2DReadOnly defaultFootPolygon;
   private final FramePoint2D icpAtStart = new FramePoint2D();
   private final FrameConvexPolygon2DBasics stancePolygon = new FrameConvexPolygon2D();

   private final RecyclingArrayList<Footstep> recoveryFootsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> recoveryFootstepTimings = new RecyclingArrayList<>(FootstepTiming::new);

   private final PushRecoveryControllerParameters pushRecoveryParameters;

   private boolean isStateCapturable = false;

   private int depth = 3;

   public MultiStepPushRecoveryCalculator(DoubleProvider kinematicsStepRange,
                                          DoubleProvider footWidth,
                                          PushRecoveryControllerParameters pushRecoveryParameters,
                                          SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                          ConvexPolygon2DReadOnly defaultFootPolygon)
   {
      this(kinematicsStepRange, kinematicsStepRange, footWidth, kinematicsStepRange, pushRecoveryParameters, soleZUpFrames, defaultFootPolygon);
   }

   public MultiStepPushRecoveryCalculator(DoubleProvider maxStepLength,
                                          DoubleProvider maxBackwardsStepLength,
                                          DoubleProvider minStepWidth,
                                          DoubleProvider maxStepWidth,
                                          PushRecoveryControllerParameters pushRecoveryParameters,
                                          SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                          ConvexPolygon2DReadOnly defaultFootPolygon)
   {
      this.soleZUpFrames = soleZUpFrames;
      this.defaultFootPolygon = defaultFootPolygon;
      this.pushRecoveryParameters = pushRecoveryParameters;
      reachableFootholdsCalculator = new ReachableFootholdsCalculator(maxStepLength,
                                                                      maxBackwardsStepLength,
                                                                      minStepWidth,
                                                                      maxStepWidth);
      captureRegionCalculator = new AchievableCaptureRegionCalculatorWithDelay();
   }

   public void setMaxStepsToGenerateForRecovery(int depth)
   {
      this.depth = depth;
   }

   public boolean computeRecoverySteps(RobotSide swingSide,
                                       double nextTransferDuration,
                                       double minSwingTimeRemaining,
                                       double maxSwingTimeRemaining,
                                       FramePoint2DReadOnly currentICP,
                                       double omega0,
                                       FrameConvexPolygon2DReadOnly footPolygon)
   {
      int numberOfRecoverySteps = calculateRecoveryStepLocations(swingSide,
                                                                 nextTransferDuration,
                                                                 minSwingTimeRemaining,
                                                                 maxSwingTimeRemaining,
                                                                 currentICP,
                                                                 omega0,
                                                                 footPolygon);

      recoveryFootsteps.clear();
      recoveryFootstepTimings.clear();

      for (int i = 0; i < numberOfRecoverySteps; i++)
      {
         stepPosition.set(recoveryStepLocations.get(i), stancePose.getZ());
         Footstep recoveryFootstep = recoveryFootsteps.add();
         recoveryFootstep.setPose(stepPosition, stancePose.getOrientation());
         recoveryFootstep.setRobotSide(swingSide);

         recoveryFootstepTimings.add().setTimings(minSwingTimeRemaining, nextTransferDuration);

         swingSide = swingSide.getOppositeSide();
      }

      return isStateCapturable;
   }

   public int getNumberOfRecoverySteps()
   {
      return recoveryFootsteps.size();
   }

   public Footstep getRecoveryStep(int stepIdx)
   {
      return recoveryFootsteps.get(stepIdx);
   }

   public void computeSquareUpStep(double preferredWidth, RobotSide nextSupportSide, Footstep squareUpStepToPack)
   {
      recoveryStepLocations.clear();
      FramePoint2DBasics squareUpLocation = recoveryStepLocations.add();
      squareUpLocation.setToZero(soleZUpFrames.get(nextSupportSide));
      squareUpLocation.changeFrame(worldFrame);

      // set square position at preferred width distance from next support foot
      squaringStepDirection.setToZero(soleZUpFrames.get(nextSupportSide));
      squaringStepDirection.add(0.0, nextSupportSide.negateIfLeftSide(preferredWidth));
      squaringStepDirection.changeFrame(worldFrame);
      squareUpPosition.set(squareUpLocation);

      squareUpStepToPack.setPose(squareUpPosition, stancePose.getOrientation());
      squareUpStepToPack.setRobotSide(nextSupportSide.getOppositeSide());
   }

   public FootstepTiming getRecoveryStepTiming(int stepIdx)
   {
      return recoveryFootstepTimings.get(stepIdx);
   }

   private int calculateRecoveryStepLocations(RobotSide swingSide,
                                              double nextTransferDuration,
                                              double minSwingTimeRemaining,
                                              double maxSwingTimeRemaining,
                                              FramePoint2DReadOnly currentICP,
                                              double omega0,
                                              FrameConvexPolygon2DReadOnly footPolygon)
   {
      icpAtStart.set(currentICP);
      stancePose.setToZero(soleZUpFrames.get(swingSide.getOppositeSide()));
      stancePose.changeFrame(worldFrame);

      int depthIdx = 0;
      stancePolygon.setIncludingFrame(footPolygon);

      int numberOfRecoverySteps = 0;
      isStateCapturable = false;

      recoveryStepLocations.clear();
      capturePointsAtTouchdown.clear();
      reachableRegions.clear();
      captureRegionsAtTouchdown.clear();
      intersectingRegions.clear();

      for (; depthIdx < depth; depthIdx++)
      {
         reachableFootholdsCalculator.calculateReachableRegion(swingSide, stancePose.getPosition(), stancePose.getOrientation(), reachableRegion);

         if (captureRegionCalculator.calculateCaptureRegion(nextTransferDuration, minSwingTimeRemaining, maxSwingTimeRemaining, icpAtStart, omega0, stancePose, stancePolygon))
         {
            isStateCapturable = true;
            break;
         }

         FrameConvexPolygon2DBasics captureRegionAtTouchdown =  captureRegionsAtTouchdown.add();

         captureRegionAtTouchdown.setMatchingFrame(captureRegionCalculator.getUnconstrainedCaptureRegion(), false);
         reachableRegions.add().set(reachableRegion);

         stancePosition.set(stancePose.getPosition());
         numberOfRecoverySteps++;

         FrameConvexPolygon2DBasics intersectingRegion = intersectingRegions.add();
         polygonTools.computeIntersectionOfPolygons(captureRegionAtTouchdown, reachableRegion, intersectingRegion);

         FramePoint2DBasics recoveryStepLocation = recoveryStepLocations.add();
         FramePoint2DBasics capturePointAtTouchdown = capturePointsAtTouchdown.add();

         if (!intersectingRegion.isEmpty())
         { // they do intersect
            FramePoint2DReadOnly centerOfIntersection = intersectingRegion.getCentroid();

            forwardLineAtNominalWidth.setToZero(stanceFrame);
            forwardLineAtNominalWidth.set(stanceFrame, 0.0, swingSide.negateIfRightSide(pushRecoveryParameters.getPreferredStepWidth()), 1.0, 0.0);
            forwardLineAtNominalWidth.changeFrame(worldFrame);

            forwardLineSegmentEnd.set(forwardLineAtNominalWidth.getPoint());
            forwardLineSegmentEnd.add(forwardLineAtNominalWidth.getDirection());
            EuclidGeometryTools.intersectionBetweenTwoLine2Ds(stancePosition, centerOfIntersection, forwardLineAtNominalWidth.getPoint(),
                                                              forwardLineSegmentEnd, capturePointAtTouchdown);

            if (!intersectingRegion.isPointInside(capturePointAtTouchdown))
            {
               EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(stancePosition,
                                                                                             centerOfIntersection,
                                                                                             intersectingRegion.getPolygonVerticesView(),
                                                                                             intersectingRegion.getNumberOfVertices(),
                                                                                             true,
                                                                                             capturePointAtTouchdown,
                                                                                             null);
            }

            recoveryStepLocation.set(capturePointAtTouchdown);
            isStateCapturable = true;
            depthIdx++;
            break;
         }
         else
         {
            polygonTools.computeMinimumDistancePoints(reachableRegion,
                                                      captureRegion,
                                                      recoveryStepLocation,
                                                      capturePointAtTouchdown);
         }

         swingSide = swingSide.getOppositeSide();


         stancePose.getPosition().set(recoveryStepLocation);
         icpAtStart.set(capturePointAtTouchdown);

         stanceFrame.setPoseAndUpdate(stancePose);
         stancePolygon.clear(stanceFrame);
         stancePolygon.set(defaultFootPolygon);
         stancePolygon.update();
         stancePolygon.scale(stancePolygon.getCentroid(), 0.5);
         stancePolygon.changeFrameAndProjectToXYPlane(worldFrame);
      }

      return numberOfRecoverySteps;
   }

   FramePoint2DReadOnly getCapturePointAtTouchdown(int i)
   {
      return capturePointsAtTouchdown.get(i);
   }

   FramePoint2DReadOnly getRecoveryStepLocation(int i)
   {
      return recoveryStepLocations.get(i);
   }

   public FrameConvexPolygon2DReadOnly getReachableRegion(int i)
   {
      return reachableRegions.get(i);
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegionAtTouchdown(int i)
   {
      return captureRegionsAtTouchdown.get(i);
   }

   public FrameConvexPolygon2DReadOnly getIntersectingRegion(int i)
   {
      return intersectingRegions.get(i);
   }
}
