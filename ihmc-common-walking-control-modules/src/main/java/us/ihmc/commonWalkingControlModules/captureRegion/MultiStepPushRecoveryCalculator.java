package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

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

   private final PoseReferenceFrame stanceFrame = new PoseReferenceFrame("StanceFrame", worldFrame);
   private final FramePose3D stancePose = new FramePose3D();
   private final FramePoint2D stancePosition = new FramePoint2D();

   private final FramePoint3D stepPosition = new FramePoint3D();
   private final FramePoint3D squareUpPosition = new FramePoint3D();

   private final ConvexPolygon2DReadOnly defaultFootPolygon;
   private final FramePoint2D icpAtStart = new FramePoint2D();
   private final FrameConvexPolygon2DBasics stancePolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2DBasics intersectingRegion = new FrameConvexPolygon2D();

   private final RecyclingArrayList<Footstep> recoveryFootsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> recoveryFootstepTimings = new RecyclingArrayList<>(FootstepTiming::new);

   private boolean isStateCapturable = false;

   private int depth = 3;

   public MultiStepPushRecoveryCalculator(DoubleProvider kinematicsStepRange,
                                          DoubleProvider footWidth,
                                          SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                          ConvexPolygon2DReadOnly defaultFootPolygon)
   {
      this(kinematicsStepRange, kinematicsStepRange, footWidth, kinematicsStepRange, soleZUpFrames, defaultFootPolygon);
   }

   public MultiStepPushRecoveryCalculator(DoubleProvider maxStepLength,
                                          DoubleProvider maxBackwardsStepLength,
                                          DoubleProvider minStepWidth,
                                          DoubleProvider maxStepWidth,
                                          SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                          ConvexPolygon2DReadOnly defaultFootPolygon)
   {
      this.soleZUpFrames = soleZUpFrames;
      this.defaultFootPolygon = defaultFootPolygon;
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
                                    double swingTimeRemaining,
                                    double nextTransferDuration,
                                    FramePoint2DReadOnly currentICP,
                                    double omega0,
                                    FrameConvexPolygon2DReadOnly footPolygon)
   {
      int numberOfRecoverySteps = calculateRecoveryStepLocations(swingSide, swingTimeRemaining, nextTransferDuration, currentICP, omega0, footPolygon);

      recoveryFootsteps.clear();
      recoveryFootstepTimings.clear();

      for (int i = 0; i < numberOfRecoverySteps; i++)
      {
         stepPosition.set(recoveryStepLocations.get(i), stancePose.getZ());
         Footstep recoveryFootstep = recoveryFootsteps.add();
         recoveryFootstep.setPose(stepPosition, stancePose.getOrientation());
         recoveryFootstep.setRobotSide(swingSide);

         recoveryFootstepTimings.add().setTimings(swingTimeRemaining, nextTransferDuration);

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

   public Footstep computeSquareUpStep(double preferredWidth, RobotSide nextSupportSide)
   {
      recoveryStepLocations.clear();
      FramePoint2DBasics squareUpLocation = recoveryStepLocations.add();

      // set square position at preferred width distance from next support foot
      squareUpLocation.setToZero(soleZUpFrames.get(nextSupportSide));
      squareUpLocation.setY(nextSupportSide.negateIfLeftSide(preferredWidth));
      squareUpLocation.changeFrame(worldFrame);
      squareUpPosition.set(squareUpLocation);

      Footstep squareUpStep = recoveryFootsteps.add();
      squareUpStep.setPose(squareUpPosition, stancePose.getOrientation());
      squareUpStep.setRobotSide(nextSupportSide.getOppositeSide());

      return squareUpStep;
   }

   public FootstepTiming getRecoveryStepTiming(int stepIdx)
   {
      return recoveryFootstepTimings.get(stepIdx);
   }

   private int calculateRecoveryStepLocations(RobotSide swingSide,
                                             double swingTimeRemaining,
                                             double nextTransferDuration,
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

      for (; depthIdx < depth; depthIdx++)
      {
         reachableFootholdsCalculator.calculateReachableRegion(swingSide, stancePose.getPosition(), stancePose.getOrientation(), reachableRegion);

         if (captureRegionCalculator.calculateCaptureRegion(swingTimeRemaining, nextTransferDuration, icpAtStart, omega0, stancePose, stancePolygon))
         {
            isStateCapturable = true;
            break;
         }

         captureRegion.setIncludingFrame(captureRegionCalculator.getUnconstrainedCaptureRegion());
         captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

         captureRegionsAtTouchdown.add().setMatchingFrame(captureRegionCalculator.getUnconstrainedCaptureRegionAtTouchdown(), false);
         reachableRegions.add().set(reachableRegion);

         stancePosition.set(stancePose.getPosition());
         numberOfRecoverySteps++;

         polygonTools.computeIntersectionOfPolygons(captureRegion, reachableRegion, intersectingRegion);

         FramePoint2DBasics recoveryStepLocation = recoveryStepLocations.add();
         FramePoint2DBasics capturePointAtTouchdown = capturePointsAtTouchdown.add();

         if (!intersectingRegion.isEmpty())
         { // they do intersect
            FramePoint2DReadOnly centerOfIntersection = intersectingRegion.getCentroid();

            EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(stancePosition,
                                                                                          centerOfIntersection,
                                                                                          intersectingRegion.getPolygonVerticesView(),
                                                                                          intersectingRegion.getNumberOfVertices(),
                                                                                          true,
                                                                                          capturePointsAtTouchdown.get(depthIdx),
                                                                                          null);

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

}
