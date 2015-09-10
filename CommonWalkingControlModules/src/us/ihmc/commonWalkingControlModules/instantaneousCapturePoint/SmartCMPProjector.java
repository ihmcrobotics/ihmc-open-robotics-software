package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Point2d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


public class SmartCMPProjector
{
   private boolean VISUALIZE = false;

   private final YoGraphicPosition icpViz, moveAwayFromEdgeViz, projectedCMPViz, preProjectedCMPViz, edgeOneViz, edgeTwoViz;
   private final BooleanYoVariable cmpProjected;
   private final DoubleYoVariable cmpEdgeProjectionInside;
   private final DoubleYoVariable minICPToCMPProjection;

   private final FrameLine2d icpToCMPLine = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2d(), new Point2d(1.0, 0.0));
   private final FramePoint2d moveAwayFromEdge = new FramePoint2d();
   private final FramePoint2d otherEdge = new FramePoint2d();
   private final FrameVector2d insideEdgeDirection = new FrameVector2d();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public SmartCMPProjector(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      cmpProjected = new BooleanYoVariable("cmpProjected", registry);
      cmpEdgeProjectionInside = new DoubleYoVariable("cmpEdgeProjectionInside", registry);
      minICPToCMPProjection = new DoubleYoVariable("minICPToCMPProjection", registry);

      cmpEdgeProjectionInside.set(0.04); //0.06);
      minICPToCMPProjection.set(0.06); //0.04);

      if (yoGraphicsListRegistry == null)
         VISUALIZE = false;

      if (VISUALIZE)
      {
         double VizBallSize = 0.3;

         icpViz = new YoGraphicPosition("icpViz", "", registry, VizBallSize, YoAppearance.Blue(), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerYoGraphic("CMPProjection", icpViz);
         yoGraphicsListRegistry.registerArtifact("CMPProjection", icpViz.createArtifact());

         moveAwayFromEdgeViz = new YoGraphicPosition("moveAwayFromEdgeViz", "", registry, VizBallSize, YoAppearance.CadetBlue(),
               GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerYoGraphic("CMPProjection", moveAwayFromEdgeViz);
         yoGraphicsListRegistry.registerArtifact("CMPProjection", moveAwayFromEdgeViz.createArtifact());

         projectedCMPViz = new YoGraphicPosition("projectedCMPViz", "", registry, VizBallSize, YoAppearance.Gold(), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerYoGraphic("CMPProjection", projectedCMPViz);
         yoGraphicsListRegistry.registerArtifact("CMPProjection", projectedCMPViz.createArtifact());

         preProjectedCMPViz = new YoGraphicPosition("preProjectedCMPViz", "", registry, VizBallSize, YoAppearance.Red(), GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("CMPProjection", preProjectedCMPViz);
         yoGraphicsListRegistry.registerArtifact("CMPProjection", preProjectedCMPViz.createArtifact());

         edgeOneViz = new YoGraphicPosition("edgeOneViz", "", registry, VizBallSize, YoAppearance.Pink(), GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("CMPProjection", edgeOneViz);
         yoGraphicsListRegistry.registerArtifact("CMPProjection", edgeOneViz.createArtifact());

         edgeTwoViz = new YoGraphicPosition("edgeTwoViz", "", registry, VizBallSize, YoAppearance.Beige(), GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("CMPProjection", edgeTwoViz);
         yoGraphicsListRegistry.registerArtifact("CMPProjection", edgeTwoViz.createArtifact());
      }
      else
      {
         icpViz = null;

         moveAwayFromEdgeViz = null;
         projectedCMPViz = null;
         preProjectedCMPViz = null;
         edgeOneViz = null;
         edgeTwoViz = null;
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setCMPEdgeProjectionInside(double cmpEdgeProjectionInside)
   {
      this.cmpEdgeProjectionInside.set(cmpEdgeProjectionInside);
   }

   public void setMinICPToCMPProjection(double minICPToCMPProjection)
   {
      this.minICPToCMPProjection.set(minICPToCMPProjection);
   }

   private final FrameVector2d cmpToICPVector = new FrameVector2d();

   
   // orders a pair of points by nearest then farthest from reference point
   public void order(FramePoint2d reference, FramePoint2d[] points)
   {
      if (points.length != 2) {
         throw new RuntimeException("requires two points, got: " + points.length);
      }

      if (reference.distance(points[0]) < reference.distance(points[1]))
         return;

      FramePoint2d temp = points[0];
      points[0] = points[1];
      points[1] = temp;
   }
   
   
   // Merges the intersections formed by projections of to collinear but opposing rays from a single point.
   // This assumes intersection with a convex polygon and removes duplicate points.
   private FramePoint2d[] merge(FramePoint2d[] one, FramePoint2d[] two, double epsilon)
   {
      if (one.length == 0) {
         return two;
      }
      
      if (two.length == 0) {
         return one;
      }
      
      if (one.length == 2) {
         return one;
      }
      
      if (two.length == 2) {
         return two;
      }
      
      if (one[0].distance(two[0]) < epsilon) {
         return one;
      }
      
      return new FramePoint2d[]{one[0], two[0]};
   }

   /**
    * Project the CMP to the support polygon by moving it along the line CMP-ICP.
    * Only problem, sometimes the line CMP-ICP doesn't intersect with the support polygon and the CMP isn't projected.
    * Risk of jumps on the CMP because of the latter.
    */
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon, FramePoint2d desiredCMP)
   {
      ReferenceFrame returnFrame = desiredCMP.getReferenceFrame();

      desiredCMP.changeFrame(supportPolygon.getReferenceFrame());
      capturePoint.changeFrame(supportPolygon.getReferenceFrame());
      cmpProjected.set(false);

      if (VISUALIZE)
      {
         projectedCMPViz.setPositionToNaN();
         edgeOneViz.setPositionToNaN();
         edgeTwoViz.setPositionToNaN();

         FramePoint2d desiredCMPInWorld = new FramePoint2d(desiredCMP);
         desiredCMPInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         preProjectedCMPViz.setPosition(desiredCMPInWorld.getX() + 0.001, desiredCMPInWorld.getY(), 0.001);

         FramePoint2d capturePointInWorld = new FramePoint2d(capturePoint);
         capturePointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         icpViz.setPosition(capturePointInWorld.getX() - 0.001, capturePointInWorld.getY(), 0.001);
      }

      boolean isCapturePointInside = supportPolygon.isPointInside(capturePoint);
      boolean isCMPInside = supportPolygon.isPointInside(desiredCMP);

      // Don't just project the cmp onto the support polygon.
      // Instead, find the first intersection from the cmp to the support polygon
      // along the line segment from the cmp to the capture point. 

      cmpToICPVector.setToZero(capturePoint.getReferenceFrame());
      cmpToICPVector.sub(capturePoint, desiredCMP);
      if (cmpToICPVector.lengthSquared() < 0.001 * 0.001)
      {
         // If CMP And ICP are really close, do nothing. Not much you can do anyway.
         desiredCMP.changeFrame(returnFrame);
         return;
      }
     
      icpToCMPLine.setIncludingFrame(capturePoint, desiredCMP);
      FramePoint2d[] icpToCMPIntersections = supportPolygon.intersectionWithRay(icpToCMPLine);
      icpToCMPLine.negateDirection();
      FramePoint2d[] icpAwayFromCMPIntersections = supportPolygon.intersectionWithRay(icpToCMPLine);
      icpToCMPLine.negateDirection();
      
      if (icpToCMPIntersections == null)
         icpToCMPIntersections = new FramePoint2d[0];
      if (icpAwayFromCMPIntersections == null)
         icpAwayFromCMPIntersections = new FramePoint2d[0];
      
      FramePoint2d[] intersections = merge(icpAwayFromCMPIntersections, icpToCMPIntersections, 1e-10);

      if (intersections.length == 0)
      {
         // If no intersections, just give up. Point is outside and no idea how to project it.
         desiredCMP.changeFrame(returnFrame);
         return;
      }

      if (VISUALIZE)
      {
         if (intersections.length > 0)
         {
            FramePoint2d intersection0InWorld = new FramePoint2d(intersections[0]);
            intersection0InWorld.changeFrame(ReferenceFrame.getWorldFrame());
            edgeOneViz.setPosition(intersection0InWorld.getX(), intersection0InWorld.getY() + 0.001, 0.0005);
         }
         else
         {
            edgeOneViz.setPositionToNaN();
         }

         if (intersections.length > 1)
         {
            FramePoint2d intersection1InWorld = new FramePoint2d(intersections[1]);
            intersection1InWorld.changeFrame(ReferenceFrame.getWorldFrame());
            edgeTwoViz.setPosition(intersection1InWorld.getX(), intersection1InWorld.getY() - 0.001, 0.0005);
         }
         else
         {
            edgeTwoViz.setPositionToNaN();
         }
      }

      if (intersections.length == 1)
      {
         // Not much you can do here. Just set the cmp to the edge and be done with it.
         desiredCMP.set(intersections[0]);
         desiredCMP.changeFrame(returnFrame);
         return;
      }
      
      if (icpAwayFromCMPIntersections.length == 2)
      {
         // special case where both are outside and on same side of polygon, just project CMP to nearest edge.
         order(desiredCMP, icpAwayFromCMPIntersections);
         desiredCMP.set(intersections[0]);
         desiredCMP.changeFrame(returnFrame);
         return;
      }

      if (isCapturePointInside)
      {
         moveAwayFromEdge.setIncludingFrame(icpToCMPIntersections[0]);
         otherEdge.setIncludingFrame(icpAwayFromCMPIntersections[0]);
      }
      else
      {
         order(capturePoint, intersections);
         moveAwayFromEdge.setIncludingFrame(intersections[1]);
         otherEdge.setIncludingFrame(intersections[0]);
      }

      insideEdgeDirection.setToZero(otherEdge.getReferenceFrame());
      insideEdgeDirection.sub(otherEdge, moveAwayFromEdge);

      double distanceFromCMPToMoveAwayFromEdge = moveAwayFromEdge.distance(desiredCMP);
      double distanceFromCMPToOtherEdge = otherEdge.distance(desiredCMP);

      if (VISUALIZE)
      {
         FramePoint2d moveAwayFromEdgeInWorld = new FramePoint2d(moveAwayFromEdge);
         moveAwayFromEdgeInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         moveAwayFromEdgeViz.setPosition(moveAwayFromEdgeInWorld.getX() - 0.002, moveAwayFromEdgeInWorld.getY() + 0.002, 0.0);
      }

      if ((isCMPInside) && (distanceFromCMPToMoveAwayFromEdge > cmpEdgeProjectionInside.getDoubleValue()))
      {
         // Point is inside and far enough away from the edge. Don't project
         desiredCMP.changeFrame(returnFrame);
         return;
      }

      if (!isCMPInside && (distanceFromCMPToOtherEdge < distanceFromCMPToMoveAwayFromEdge))
      {
         // Point is outside but close to ICP, just project to ICP
         desiredCMP.set(otherEdge);
         desiredCMP.changeFrame(returnFrame);
         return;
      }

      // Stay cmpEdgeProjectionDistance away from the edge if possible.
      // By possible, we mean if you were to move inside by cmpEdgeProjectionDistance,
      // Make sure you are still at least minCMPProjectionDistance from the ICP,
      // unless that would put you over the edge. Then just use the edge and hope for the best.
      cmpProjected.set(true);
      double distanceFromICPToMoveAwayFromEdge = moveAwayFromEdge.distance(capturePoint);
      double distanceToMove = distanceFromICPToMoveAwayFromEdge - minICPToCMPProjection.getDoubleValue();

      if (isCMPInside)
      {
         double distanceToCMP = moveAwayFromEdge.distance(desiredCMP);
         if (distanceToMove < distanceToCMP)
            distanceToMove = distanceToCMP;
      }

      if (distanceToMove < 0.0)
         distanceToMove = 0.0;
      if (distanceToMove > cmpEdgeProjectionInside.getDoubleValue())
         distanceToMove = cmpEdgeProjectionInside.getDoubleValue();

      double edgeToEdgeDistance = moveAwayFromEdge.distance(otherEdge);
      if (distanceToMove > edgeToEdgeDistance)
         distanceToMove = edgeToEdgeDistance;
      insideEdgeDirection.normalize();
      insideEdgeDirection.scale(distanceToMove);

      desiredCMP.setIncludingFrame(moveAwayFromEdge);
      desiredCMP.add(insideEdgeDirection);

      if (VISUALIZE)
      {
         FramePoint2d desiredCMPInWorld = new FramePoint2d(desiredCMP);
         desiredCMPInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         projectedCMPViz.setPosition(desiredCMPInWorld.getX(), desiredCMPInWorld.getY(), 0.0);
      }

      desiredCMP.changeFrame(returnFrame);
   }



   public boolean getWasCMPProjected()
   {
      return cmpProjected.getBooleanValue();
   }

}
