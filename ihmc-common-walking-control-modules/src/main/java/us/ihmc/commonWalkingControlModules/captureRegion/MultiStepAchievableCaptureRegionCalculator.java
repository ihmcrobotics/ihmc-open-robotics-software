package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class MultiStepAchievableCaptureRegionCalculator
{
   private static final boolean VISUALIZE = true;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ReachableFootholdsCalculator reachableFootholdsCalculator;
   private final AchievableCaptureRegionCalculatorWithDelay captureRegionCalculator;

   private final RecyclingArrayList<FramePoint2DBasics> currentICPVertices = new RecyclingArrayList<>(FramePoint2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2DBasics> captureRegions = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final List<YoFrameConvexPolygon2D> yoCaptureRegions = new ArrayList<>();


   private final int depth = 3;

   public MultiStepAchievableCaptureRegionCalculator(double kinematicsStepRange,
                                                     double footWidth,
                                                     SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                                     String suffix,
                                                     YoRegistry parentRegistry,
                                                     YoGraphicsListRegistry graphicsListRegistry)
   {
      reachableFootholdsCalculator = new ReachableFootholdsCalculator(kinematicsStepRange, kinematicsStepRange, 0.5 * footWidth, kinematicsStepRange,
                                                                      soleZUpFrames, suffix, registry, graphicsListRegistry);
      captureRegionCalculator = new AchievableCaptureRegionCalculatorWithDelay(footWidth, kinematicsStepRange, soleZUpFrames,
                                                                               suffix, registry, null);

      parentRegistry.addChild(registry);
      if (graphicsListRegistry != null && VISUALIZE)
      {
         for (int i = 0; i < depth; i++)
         {
            String name = "captureRegion" + i;

            YoFrameConvexPolygon2D yoCaptureRegionPolygon = new YoFrameConvexPolygon2D(name, suffix, worldFrame, 30, registry);
            yoCaptureRegions.add(yoCaptureRegionPolygon);

            YoArtifactPolygon polygonArtifact = new YoArtifactPolygon(name + suffix, yoCaptureRegionPolygon, Color.YELLOW, false);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact);
         }
      }
   }

   public void calculateCaptureRegions(RobotSide swingSide,
                                       double swingTimeRemaining,
                                       double nextTransferDuration,
                                       FramePoint2DReadOnly currentICP,
                                       double omega0,
                                       FrameConvexPolygon2DReadOnly footPolygon)
   {
      reachableFootholdsCalculator.calculateReachableRegions(swingSide);

      currentICPVertices.clear();
      currentICPVertices.add().setIncludingFrame(currentICP);

      int depthIdx = 0;
      FrameConvexPolygon2DReadOnly polygon = footPolygon;
      for (; depthIdx < depth; depthIdx++)
      {
         FrameConvexPolygon2DReadOnly captureRegion = computeCaptureRegionFromRegion(swingSide, swingTimeRemaining, nextTransferDuration, currentICPVertices, omega0, polygon);
         if (VISUALIZE)
            yoCaptureRegions.get(depthIdx).set(captureRegion);

         swingSide = swingSide.getOppositeSide();
         polygon = reachableFootholdsCalculator.getReachableRegion(depthIdx);

         currentICPVertices.clear();
         for (int vertexIdx = 0; vertexIdx < captureRegion.getNumberOfVertices(); vertexIdx++)
            currentICPVertices.add().set(captureRegion.getVertex(vertexIdx));
      }

      if (VISUALIZE)
      {
         for (; depthIdx < depth; depthIdx++)
            yoCaptureRegions.get(depthIdx).clearAndUpdate();
      }
   }

   private FrameConvexPolygon2DReadOnly computeCaptureRegionFromRegion(RobotSide swingSide,
                                               double swingTimeRemaining,
                                               double nextTransferDuration,
                                               List<? extends FramePoint2DReadOnly> currentICPVertices,
                                               double omega0,
                                               FrameConvexPolygon2DReadOnly cmpVertices)
   {
      FrameConvexPolygon2DBasics captureRegion = captureRegions.add();
      captureRegion.clear();
      for (int i = 0; i < currentICPVertices.size(); i++)
      {
         captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, nextTransferDuration, currentICPVertices.get(i), omega0, cmpVertices);
         FrameConvexPolygon2DReadOnly unconstrainedRegion = captureRegionCalculator.getUnconstrainedCaptureRegion();
         for (int regionVertexIdx = 0; regionVertexIdx < unconstrainedRegion.getNumberOfVertices(); regionVertexIdx++)
         {
            captureRegion.addVertexMatchingFrame(unconstrainedRegion.getVertex(regionVertexIdx), false);
         }
      }
      captureRegion.update();

      return captureRegion;
   }

}
