package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class MultiStepAchievableCaptureRegionCalculator
{
   private static final boolean VISUALIZE = true;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;

   private final ReachableFootholdsCalculator reachableFootholdsCalculator;
   private final AchievableCaptureRegionCalculatorWithDelay captureRegionCalculator;

   private final ConvexPolygonTools polygonTools = new ConvexPolygonTools();

   private final FrameConvexPolygon2D reachableRegion = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();

   private final List<YoFramePoint2D> capturePointsAtTouchdown = new ArrayList<>();
   private final List<YoFramePoint2D> recoveryStepLocations = new ArrayList<>();

   private final List<YoFrameConvexPolygon2D> yoCaptureRegions = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> yoReachableRegions = new ArrayList<>();

   private static final int maxRegionDepth = 3;

   private final int depth = 3;

   public MultiStepAchievableCaptureRegionCalculator(double kinematicsStepRange,
                                                     double footWidth,
                                                     SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                                     String suffix,
                                                     YoRegistry parentRegistry,
                                                     YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;
      reachableFootholdsCalculator = new ReachableFootholdsCalculator(kinematicsStepRange,
                                                                      kinematicsStepRange,
                                                                      0.5 * footWidth,
                                                                      kinematicsStepRange,
                                                                      soleZUpFrames,
                                                                      registry);
      captureRegionCalculator = new AchievableCaptureRegionCalculatorWithDelay(footWidth, kinematicsStepRange, soleZUpFrames, suffix, registry, null);

      for (int i = 0; i < maxRegionDepth; i++)
      {
         capturePointsAtTouchdown.add(new YoFramePoint2D("capturePointAtTouchdown" + i, worldFrame, registry));
         recoveryStepLocations.add(new YoFramePoint2D("recoveryStepLocation" + i, worldFrame, registry));
      }

      if (graphicsListRegistry != null && VISUALIZE)
      {
         String listName = getClass().getSimpleName();
         for (int i = 0; i < maxRegionDepth; i++)
         {
            String captureName = "captureRegion" + i;

            YoFrameConvexPolygon2D yoCaptureRegionPolygon = new YoFrameConvexPolygon2D(captureName, suffix, worldFrame, 30, registry);
            yoCaptureRegions.add(yoCaptureRegionPolygon);

            YoArtifactPolygon capturePolygonArtifact = new YoArtifactPolygon(captureName + suffix, yoCaptureRegionPolygon, Color.YELLOW, false);

            String reachableName = "reachableRegion" + i;

            YoFrameConvexPolygon2D yoReachableRegionPolygon = new YoFrameConvexPolygon2D(reachableName, suffix, worldFrame, 30, registry);
            yoReachableRegions.add(yoReachableRegionPolygon);

            YoArtifactPolygon reachablePolygonArtifact = new YoArtifactPolygon(reachableName + suffix, yoReachableRegionPolygon, Color.BLUE, false);

            YoGraphicPosition touchdownICPViz = new YoGraphicPosition("capturePointTouchdown" + i + suffix, capturePointsAtTouchdown.get(i), 0.01, YoAppearance.Yellow(),
                                                                   YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
            YoGraphicPosition footstepViz = new YoGraphicPosition("recoveryStepLocation" + i + suffix, recoveryStepLocations.get(i), 0.01, YoAppearance.Blue(),
                                                                      YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

            graphicsListRegistry.registerArtifact(listName, capturePolygonArtifact);
            graphicsListRegistry.registerArtifact(listName, reachablePolygonArtifact);
            graphicsListRegistry.registerArtifact(listName, touchdownICPViz.createArtifact());
            graphicsListRegistry.registerArtifact(listName, footstepViz.createArtifact());
         }
      }

      parentRegistry.addChild(registry);

   }

   private final FramePose3D stancePose = new FramePose3D();
   private final FramePoint2D stancePosition = new FramePoint2D();

   private final FramePoint2D icpAtStart = new FramePoint2D();
   private final FrameConvexPolygon2DBasics stancePolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2DBasics intersectingRegion = new FrameConvexPolygon2D();

   public int calculateRecoveryStepLocations(RobotSide swingSide,
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

      for (; depthIdx < depth; depthIdx++)
      {
         reachableFootholdsCalculator.calculateReachableRegion(swingSide, stancePose.getPosition(), stancePose.getOrientation(), reachableRegion);

         captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, nextTransferDuration, icpAtStart, omega0, stancePolygon);
         captureRegion.setIncludingFrame(captureRegionCalculator.getUnconstrainedCaptureRegion());
         captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

         if (VISUALIZE)
         {
            yoCaptureRegions.get(depthIdx).set(captureRegion);
            yoReachableRegions.get(depthIdx).set(reachableRegion);
         }

         stancePosition.set(stancePose.getPosition());
         numberOfRecoverySteps++;

         if (polygonTools.computeIntersectionOfPolygons(captureRegion, reachableRegion, intersectingRegion))
         { // they do intersect
            FramePoint2DReadOnly centerOfIntersection = intersectingRegion.getCentroid();

            EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(stancePosition,
                                                                                          centerOfIntersection,
                                                                                          intersectingRegion.getPolygonVerticesView(),
                                                                                          intersectingRegion.getNumberOfVertices(),
                                                                                          true,
                                                                                          capturePointsAtTouchdown.get(depthIdx),
                                                                                          null);

            recoveryStepLocations.get(depthIdx).set(capturePointsAtTouchdown.get(depthIdx));
            depthIdx++;
            break;
         }
         else
         {
            polygonTools.computeMinimumDistancePoints(reachableRegion, captureRegion, recoveryStepLocations.get(depthIdx), capturePointsAtTouchdown.get(depthIdx));
         }

         swingSide = swingSide.getOppositeSide();
         stancePolygon.clear();
         stancePolygon.addVertex(recoveryStepLocations.get(depthIdx));
         stancePolygon.update();

         stancePose.getPosition().set(recoveryStepLocations.get(depthIdx));
         icpAtStart.set(capturePointsAtTouchdown.get(depthIdx));
      }

      if (VISUALIZE)
      {
         for (; depthIdx < depth; depthIdx++)
         {
            capturePointsAtTouchdown.get(depthIdx).setToNaN();
            recoveryStepLocations.get(depthIdx).setToNaN();
            yoReachableRegions.get(depthIdx).clearAndUpdate();
            yoCaptureRegions.get(depthIdx).clearAndUpdate();
         }
      }

      return numberOfRecoverySteps;
   }
}
