package us.ihmc.commonWalkingControlModules.captureRegion;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntFunction;

public class MultiStepRecoveryStepCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private final ReachableFootholdsCalculator reachableFootholdsCalculator;
   private final AchievableCaptureRegionCalculatorWithDelay captureRegionCalculator;

   private final ConvexPolygonTools polygonTools = new ConvexPolygonTools();

   private final FrameConvexPolygon2D reachableRegion = new FrameConvexPolygon2D();

   private final RecyclingArrayList<FramePoint2DBasics> capturePointsAtTouchdown = new RecyclingArrayList<>(FramePoint2D::new);
   private final RecyclingArrayList<FramePoint2DBasics> recoveryStepLocations = new RecyclingArrayList<>(FramePoint2D::new);

   private final RecyclingArrayList<ConstrainedReachableRegion> constrainedReachableRegions = new RecyclingArrayList<>(ConstrainedReachableRegion::new);

   private final RecyclingArrayList<FrameConvexPolygon2DBasics> captureRegionsAtTouchdown = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2DBasics> reachableRegions = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2DBasics> reachableCaptureRegions = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2DBasics> reachableConstrainedCaptureRegions = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final List<ConstrainedReachableRegion> constraintRegions = new ArrayList<>();

   private final PoseReferenceFrame stanceFrame = new PoseReferenceFrame("StanceFrame", worldFrame);
   private final FramePose3D stancePose = new FramePose3D();
   private final FramePoint2D stancePosition = new FramePoint2D();

   private final FrameLine2D forwardLineAtNominalWidth = new FrameLine2D();
   private final FramePoint2D forwardLineSegmentEnd = new FramePoint2D();

   private final FramePoint3D stepPosition = new FramePoint3D();

   private final ConvexPolygon2DReadOnly defaultFootPolygon;
   private final FramePoint2D icpAtStart = new FramePoint2D();
   private final FrameConvexPolygon2DBasics stancePolygon = new FrameConvexPolygon2D();

   private final RecyclingArrayList<Footstep> recoveryFootsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> recoveryFootstepTimings = new RecyclingArrayList<>(FootstepTiming::new);

   private final PushRecoveryControllerParameters pushRecoveryParameters;
   private IntFunction<List<StepConstraintRegion>> constraintRegionProvider;

   private boolean isStateCapturable = false;

   private int depth = 3;

   public MultiStepRecoveryStepCalculator(DoubleProvider kinematicsStepRange,
                                          DoubleProvider footWidth,
                                          PushRecoveryControllerParameters pushRecoveryParameters,
                                          SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                          ConvexPolygon2DReadOnly defaultFootPolygon)
   {
      this(kinematicsStepRange, kinematicsStepRange, footWidth, kinematicsStepRange, pushRecoveryParameters, soleZUpFrames, defaultFootPolygon);
   }

   public MultiStepRecoveryStepCalculator(DoubleProvider maxStepLength,
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

      reachableFootholdsCalculator = new ReachableFootholdsCalculator(maxStepLength, maxBackwardsStepLength, minStepWidth, maxStepWidth);
      captureRegionCalculator = new AchievableCaptureRegionCalculatorWithDelay();
   }

   public void setConstraintRegionProvider(IntFunction<List<StepConstraintRegion>> constraintRegionProvider)
   {
      this.constraintRegionProvider = constraintRegionProvider;
   }

   public void setMaxStepsToGenerateForRecovery(int depth)
   {
      this.depth = depth;
   }

   private final TDoubleArrayList candidateSwingTimes = new TDoubleArrayList();

   public boolean computePreferredRecoverySteps(RobotSide swingSide,
                                                double nextTransferDuration,
                                                double swingTimeRemaining,
                                                FramePoint2DReadOnly currentICP,
                                                double omega0,
                                                FrameConvexPolygon2DReadOnly footPolygon)
   {
      candidateSwingTimes.reset();
      candidateSwingTimes.add(swingTimeRemaining);

      return computeRecoverySteps(swingSide, nextTransferDuration, swingTimeRemaining, candidateSwingTimes, currentICP, omega0, footPolygon);
   }

   public boolean computeRecoverySteps(RobotSide swingSide,
                                       double nextTransferDuration,
                                       double minSwingTimeRemaining,
                                       double maxSwingTimeRemaining,
                                       FramePoint2DReadOnly currentICP,
                                       double omega0,
                                       FrameConvexPolygon2DReadOnly footPolygon)
   {
      candidateSwingTimes.reset();
      candidateSwingTimes.add(minSwingTimeRemaining);
      candidateSwingTimes.add(maxSwingTimeRemaining);

      return computeRecoverySteps(swingSide, nextTransferDuration, minSwingTimeRemaining, candidateSwingTimes, currentICP, omega0, footPolygon);
   }

   public boolean computeRecoverySteps(RobotSide swingSide,
                                       double nextTransferDuration,
                                       double swingTimeToSet,
                                       TDoubleArrayList candidateSwingTimes,
                                       FramePoint2DReadOnly currentICP,
                                       double omega0,
                                       FrameConvexPolygon2DReadOnly footPolygon)
   {
      int numberOfRecoverySteps = calculateRecoveryStepLocations(swingSide, nextTransferDuration, candidateSwingTimes, currentICP, omega0, footPolygon);

      recoveryFootsteps.clear();
      recoveryFootstepTimings.clear();

      for (int i = 0; i < numberOfRecoverySteps; i++)
      {
         stepPosition.set(recoveryStepLocations.get(i), stancePose.getZ());
         Footstep recoveryFootstep = recoveryFootsteps.add();
         recoveryFootstep.setPose(stepPosition, stancePose.getOrientation());
         recoveryFootstep.setRobotSide(swingSide);

         recoveryFootstepTimings.add().setTimings(swingTimeToSet, nextTransferDuration);

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

   public FootstepTiming getRecoveryStepTiming(int stepIdx)
   {
      return recoveryFootstepTimings.get(stepIdx);
   }

   private final FramePoint2D pointToThrowAway = new FramePoint2D();

   private int calculateRecoveryStepLocations(RobotSide swingSide,
                                              double nextTransferDuration,
                                              TDoubleArrayList candidateSwingTimes,
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
      reachableCaptureRegions.clear();
      reachableConstrainedCaptureRegions.clear();

      constraintRegions.clear();

      for (; depthIdx < depth; depthIdx++)
      {
         reachableFootholdsCalculator.calculateReachableRegion(swingSide, stancePose.getPosition(), stancePose.getOrientation(), reachableRegion);

         if (captureRegionCalculator.calculateCaptureRegion(nextTransferDuration, candidateSwingTimes, icpAtStart, omega0, stancePose, stancePolygon))
         {
            isStateCapturable = true;
            break;
         }

         FrameConvexPolygon2DBasics captureRegionAtTouchdown = captureRegionsAtTouchdown.add();

         captureRegionAtTouchdown.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);
         reachableRegions.add().set(reachableRegion);

         stancePosition.set(stancePose.getPosition());
         numberOfRecoverySteps++;

         FrameConvexPolygon2DBasics reachableCaptureRegion = reachableCaptureRegions.add();
         polygonTools.computeIntersectionOfPolygons(captureRegionAtTouchdown, reachableRegion, reachableCaptureRegion);

         FramePoint2DBasics recoveryStepLocation = recoveryStepLocations.add();
         FramePoint2DBasics capturePointAtTouchdown = capturePointsAtTouchdown.add();

         computeConstrainedReachableRegions(depthIdx, reachableRegion);

         FrameConvexPolygon2DBasics constrainedCaptureRegion = reachableConstrainedCaptureRegions.add();
         computeConstrainedCaptureRegion(reachableCaptureRegion, constrainedCaptureRegion);

         if (!constrainedCaptureRegion.isEmpty())
         { // they do intersect
            FramePoint2DReadOnly centerOfIntersection = constrainedCaptureRegion.getCentroid();

            computeRecoveryStepAtNominalWidth(swingSide, stancePosition, centerOfIntersection, capturePointAtTouchdown);

            if (!constrainedCaptureRegion.isPointInside(capturePointAtTouchdown))
            {
               EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(stancePosition,
                                                                                             centerOfIntersection,
                                                                                             constrainedCaptureRegion.getPolygonVerticesView(),
                                                                                             constrainedCaptureRegion.getNumberOfVertices(),
                                                                                             true,
                                                                                             capturePointAtTouchdown,
                                                                                             pointToThrowAway);
            }

            recoveryStepLocation.set(capturePointAtTouchdown);
            isStateCapturable = true;
            depthIdx++;
            break;
         }
         else
         {
            if (constraintRegionProvider == null || constraintRegionProvider.apply(depthIdx) == null || constrainedReachableRegions.size() == 0)
            {
               isStateCapturable = computeUnconstrainedBestEffortStep(captureRegionAtTouchdown, reachableRegion, capturePointAtTouchdown, recoveryStepLocation);
            }
            else
            {
               isStateCapturable = computeBestEffortStepWithEnvironmentalConstrains(captureRegionAtTouchdown,
                                                                                    capturePointAtTouchdown,
                                                                                    recoveryStepLocation);
            }
            if (!isStateCapturable)
               break;
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

   private void computeRecoveryStepAtNominalWidth(RobotSide swingSide,
                                                  FramePoint2DReadOnly stancePosition,
                                                  FramePoint2DReadOnly pointInDirectionOfStep,
                                                  FramePoint2DBasics recoveryStepLocationToPack)
   {

      forwardLineAtNominalWidth.setToZero(stanceFrame);
      forwardLineAtNominalWidth.set(stanceFrame, 0.0, swingSide.negateIfRightSide(pushRecoveryParameters.getPreferredStepWidth()), 1.0, 0.0);
      forwardLineAtNominalWidth.changeFrame(worldFrame);

      forwardLineSegmentEnd.set(forwardLineAtNominalWidth.getPoint());
      forwardLineSegmentEnd.add(forwardLineAtNominalWidth.getDirection());
      EuclidGeometryTools.intersectionBetweenTwoLine2Ds(stancePosition,
                                                        pointInDirectionOfStep,
                                                        forwardLineAtNominalWidth.getPoint(),
                                                        forwardLineSegmentEnd,
                                                        recoveryStepLocationToPack);
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
      return reachableCaptureRegions.get(i);
   }

   public boolean hasConstraintRegions()
   {
      return !constraintRegions.isEmpty();
   }

   public StepConstraintRegion getConstraintRegionInWorld(int i)
   {
      if (constraintRegions.get(i) != null)
         return constraintRegions.get(i).getStepConstraintRegion();
      return null;
   }

   private void computeConstrainedCaptureRegion(FrameConvexPolygon2DReadOnly reachableCaptureRegion,
                                                FrameConvexPolygon2DBasics constrainedCaptureRegionToPack)
   {
      if (constrainedReachableRegions.size() == 0)
      {
         constrainedCaptureRegionToPack.set(reachableCaptureRegion);
         return;
      }

      ConstrainedReachableRegion constraintRegion = null;
      if (constrainedReachableRegions.size() == 1)
      {
         constraintRegion = constrainedReachableRegions.get(0);
      }
      else
      {
         double minArea = 0.0;
         for (int i = 0; i < constrainedReachableRegions.size(); i++)
         {
            ConstrainedReachableRegion region = constrainedReachableRegions.get(i);
            double area = polygonTools.computeIntersectionAreaOfPolygons(reachableCaptureRegion, region);
            if (area > minArea)
            {
               minArea = area;
               constraintRegion = region;
            }
         }
      }

      constraintRegions.add(constraintRegion);

      if (constraintRegion != null)
      {
         polygonTools.computeIntersectionOfPolygons(reachableCaptureRegion, constraintRegion, constrainedCaptureRegionToPack);
      }
      else
      {
         constrainedCaptureRegionToPack.clearAndUpdate();
      }
   }

   private boolean computeUnconstrainedBestEffortStep(FrameConvexPolygon2DReadOnly captureRegion,
                                                      FrameConvexPolygon2DReadOnly reachableRegion,
                                                      FramePoint2DBasics capturePointAtTouchdownToPack,
                                                      FramePoint2DBasics recoveryStepLocationToPack)
   {
      if (captureRegion.getNumberOfVertices() == 0)
      {
         return false;
      }
      else if (captureRegion.getNumberOfVertices() == 1)
      {
         capturePointAtTouchdownToPack.set(captureRegion.getVertex(0));
         reachableRegion.orthogonalProjection(capturePointAtTouchdownToPack, recoveryStepLocationToPack);
      }
      else if (captureRegion.getNumberOfVertices() == 2)
      {
         throw new RuntimeException("The event of the capture region having two points still needs to be implemented.");
      }
      else
      {
         polygonTools.computeMinimumDistancePoints(reachableRegion, captureRegion, recoveryStepLocationToPack, capturePointAtTouchdownToPack);
      }

      return true;
   }

   private final FramePoint2D point1ToThrowAway = new FramePoint2D();
   private final FramePoint2D point2ToThrowAway = new FramePoint2D();

   private boolean computeBestEffortStepWithEnvironmentalConstrains(FrameConvexPolygon2DReadOnly captureRegion,
                                                                    FramePoint2DBasics capturePointAtTouchdownToPack,
                                                                    FramePoint2DBasics recoveryStepLocationToPack)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      boolean isStateCapturable = false;

      for (int i = 0; i < constrainedReachableRegions.size(); i++)
      {
         FrameConvexPolygon2DReadOnly constrainedReachableRegion = constrainedReachableRegions.get(i);

         if (constrainedReachableRegion.isEmpty())
            continue;

         boolean regionIsCapturable = computeUnconstrainedBestEffortStep(captureRegion, constrainedReachableRegion, point1ToThrowAway, point2ToThrowAway);
         if (regionIsCapturable)
            isStateCapturable = true;

         double distanceSquared = point1ToThrowAway.distanceSquared(point2ToThrowAway);
         if (!isStateCapturable || regionIsCapturable && (distanceSquared < closestDistance))
         {
            closestDistance = distanceSquared;
            capturePointAtTouchdownToPack.set(point1ToThrowAway);
            recoveryStepLocationToPack.set(point2ToThrowAway);
         }
      }

      return isStateCapturable;
   }

   private final FrameConvexPolygon2D constraintRegionHull = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D intersectingHull = new FrameConvexPolygon2D();

   private void computeConstrainedReachableRegions(int stepNumber, FrameConvexPolygon2DReadOnly reachableRegion)
   {
      constrainedReachableRegions.clear();
      if (constraintRegionProvider == null || constraintRegionProvider.apply(stepNumber) == null)
         return;

      List<StepConstraintRegion> constraintRegions = constraintRegionProvider.apply(stepNumber);
      for (int i = 0; i < constraintRegions.size(); i++)
      {
         constraintRegionHull.set(constraintRegions.get(i).getConvexHull());
         constraintRegionHull.applyTransform(constraintRegions.get(i).getTransformToWorld(), false);

         polygonTools.computeIntersectionOfPolygons(reachableRegion, constraintRegionHull, intersectingHull);
         if (intersectingHull.getArea() > 0.0)
         {
            ConstrainedReachableRegion constrainedReachableRegion = constrainedReachableRegions.add();
            constrainedReachableRegion.set(intersectingHull);
            constrainedReachableRegion.setStepConstraintRegion(constraintRegions.get(i));
         }
      }
   }

   private static class ConstrainedReachableRegion extends FrameConvexPolygon2D
   {
      private StepConstraintRegion stepConstraintRegion;

      public void setStepConstraintRegion(StepConstraintRegion stepConstraintRegion)
      {
         this.stepConstraintRegion = stepConstraintRegion;
      }

      public StepConstraintRegion getStepConstraintRegion()
      {
         return stepConstraintRegion;
      }
   }
}
