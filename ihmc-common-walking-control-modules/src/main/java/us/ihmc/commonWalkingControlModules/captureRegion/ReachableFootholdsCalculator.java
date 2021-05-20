package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class ReachableFootholdsCalculator
{
   private static final boolean VISUALIZE = true;
   private static final int numberOfVertices = 5;

   private static final int maxRegionDepth = 3;
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;
   private final PoseReferenceFrame rotatedSoleFrame = new PoseReferenceFrame("rotatedSoleFrame", ReferenceFrame.getWorldFrame());


   private final int regionDepth = 3;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final RecyclingArrayList<FrameConvexPolygon2D> reachableRegions = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final List<YoFrameConvexPolygon2D> yoReachableRegions = new ArrayList<>();

   private final YoDouble lengthLimit;
   private final YoDouble lengthBackLimit;
   private final YoDouble innerLimit;
   private final YoDouble outerLimit;

   public ReachableFootholdsCalculator(double maxStepLength,
                                       double maxBackwardStepLength,
                                       double minStepWidth,
                                       double maxStepWidth,
                                       SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                       String suffix,
                                       YoRegistry parentRegistry,
                                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;
      //      this.midFootAnkleXOffset = midFootAnkleXOffset;

      lengthLimit = new YoDouble("MaxReachabilityLength", registry);
      lengthBackLimit = new YoDouble("MaxReachabilityBackwardLength", registry);
      innerLimit = new YoDouble("MinReachabilityWidth", registry);
      outerLimit = new YoDouble("MaxReachabilityWidth", registry);

      lengthLimit.set(maxStepLength);
      lengthBackLimit.set(maxBackwardStepLength);
      innerLimit.set(minStepWidth);
      outerLimit.set(maxStepWidth);

      parentRegistry.addChild(registry);

      // set up registry and visualizer
      //      parentRegistry.addChild(registry);
      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         for (int i = 0; i < maxRegionDepth; i++)
         {
            String name = "reachableRegion" + i;

            YoFrameConvexPolygon2D yoCaptureRegionPolygon = new YoFrameConvexPolygon2D(name, suffix, worldFrame, 30, registry);
            yoReachableRegions.add(yoCaptureRegionPolygon);

            YoArtifactPolygon polygonArtifact = new YoArtifactPolygon(name + suffix, yoCaptureRegionPolygon, Color.BLUE, false);
            yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact);
         }
      }
   }

   private final RecyclingArrayList<FramePoint2D> origins = new RecyclingArrayList<>(FramePoint2D::new);

   public void calculateReachableRegions(RobotSide swingSide)
   {
      rotatedSoleFrame.setOrientationAndUpdate(soleZUpFrames.get(swingSide.getOppositeSide()).getTransformToParent().getRotation());

      origins.clear();
      FramePoint2D origin = origins.add();
      origin.setToZero(soleZUpFrames.get(swingSide.getOppositeSide()));
      origin.changeFrame(ReferenceFrame.getWorldFrame());

      int regionIdx = 0;
      for (; regionIdx < regionDepth; regionIdx++)
      {
         FrameConvexPolygon2DReadOnly reachableRegion = calculateReachableRegion(origins, swingSide);
         if (VISUALIZE)
            yoReachableRegions.get(regionIdx).set(reachableRegion);

         swingSide = swingSide.getOppositeSide();
         origins.clear();
         for (int vertexIdx = 0; vertexIdx < reachableRegion.getNumberOfVertices(); vertexIdx++)
            origins.add().set(reachableRegions.get(regionIdx).getVertex(vertexIdx));
      }

      if (VISUALIZE)
      {
         for (; regionIdx < maxRegionDepth; regionIdx++)
            yoReachableRegions.get(regionIdx).clearAndUpdate();
      }
   }

   public FrameConvexPolygon2DReadOnly getReachableRegion(int regionIdx)
   {
      return reachableRegions.get(regionIdx);
   }

   private final FramePoint2D tempPoint = new FramePoint2D();

   private FrameConvexPolygon2DReadOnly calculateReachableRegion(List<? extends FramePoint2DReadOnly> origins, RobotSide swingSide)
   {
      FrameConvexPolygon2D reachableRegion = reachableRegions.add();
      reachableRegion.clear();

      double sign = swingSide.negateIfRightSide(1.0);

      for (int originIdx = 0; originIdx < origins.size(); originIdx++)
      {
         FramePoint2DReadOnly origin = origins.get(originIdx);


         // create an ellipsoid around the center of the forward and backward reachable limits
         double xRadius = 0.5 * (lengthLimit.getValue() + lengthBackLimit.getValue());
         double yRadius = outerLimit.getValue() - innerLimit.getValue();
         double centerX = lengthLimit.getValue() - xRadius;
         double centerY = innerLimit.getValue();

         // compute the vertices on the edge of the ellipsoid
         for (int vertexIdx = 0; vertexIdx < numberOfVertices; vertexIdx++)
         {
            double angle = Math.PI * vertexIdx / (numberOfVertices - 1);
            double x = centerX + xRadius * Math.cos(angle);
            double y = centerY + yRadius * Math.sin(angle);

            tempPoint.setIncludingFrame(rotatedSoleFrame, x, sign * y);
            tempPoint.changeFrame(worldFrame);
            tempPoint.add(origin);
            reachableRegion.addVertex(tempPoint);
         }
      }
      reachableRegion.update();

      return reachableRegion;
   }
}
