package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

/**
 * This class is meant to shrink the capture region in an intelligent way, using three safety margins. The first two compute the line going from
 * the center of the stance foot through the current capture point. They then project the vertices of the capture region inward towards this line.
 * The vertices that are on the "inside" of the region (the side that face the stance foot) get shrunk more using the first margin
 * ({@link #distanceIntoCaptureRegionForInside}). All vertices get shrunk by a minimal amount ({@link #distanceIntoCaptureRegionForEverywhere}). The
 * amount that the inside regions get shrunk by is reduced as the "line" goes towards a sideways step. Then, all the vertices of the capture region
 * that are visible to the stance foot are projected further by {@link #extraDistanceToStepFromStanceFoot}. This forces the robot to take longer steps,
 * rather than just taking the shortest step.
 */
public class CaptureRegionSafetyHeuristics
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FrameLine2D lineOfMinimalAction = new FrameLine2D();
   private final FrameConvexPolygon2D saferCaptureRegion = new FrameConvexPolygon2D();

   private final YoFrameLine2D yoLineOfMinimalAction = new YoFrameLine2D("yoLineOfMinimalAction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameConvexPolygon2D yoSafetyBiasedCaptureRegion = new YoFrameConvexPolygon2D("safetyBiasedCaptureRegion",
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 30,
                                                                                                 registry);

   private final DoubleParameter distanceIntoCaptureRegionForInside = new DoubleParameter("distanceIntoCaptureRegionForInside", registry, 0.05);
   private final DoubleParameter distanceIntoCaptureRegionForEverywhere = new DoubleParameter("distanceIntoCaptureRegionForEverywhere", registry, 0.02);
   private final DoubleParameter extraDistanceToStepFromStanceFoot = new DoubleParameter("extraDistanceToStepFromStanceFoot", registry, 0.05);

   private final List<FixedFramePoint2DBasics> verticesVisibleFromStance = new ArrayList<>();
   private final FramePoint2D stancePosition = new FramePoint2D();

   private final FrameVector2D vectorToVertex = new FrameVector2D();
   private final FramePoint2D projectedPoint = new FramePoint2D();

   private final DoubleProvider reachabilityLimit;

   private final FrameVector2D forwardVector = new FrameVector2D();
   private final FrameVector2D tempVector = new FrameVector2D();
   private final FramePoint2D croppedPoint = new FramePoint2D();

   public CaptureRegionSafetyHeuristics(DoubleProvider reachabilityLimit, YoRegistry parentRegistry)
   {
      this(reachabilityLimit, parentRegistry, null);
   }

   public CaptureRegionSafetyHeuristics(DoubleProvider reachabilityLimit, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.reachabilityLimit = reachabilityLimit;

      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon safePolygonArtifact = new YoArtifactPolygon("Safety Biased Capture Region", yoSafetyBiasedCaptureRegion, Color.GREEN, false, true);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), safePolygonArtifact);
      }

      parentRegistry.addChild(registry);
   }

   /**
    * Clears the visualizer.
    */
   public void reset()
   {
      yoSafetyBiasedCaptureRegion.clear();
   }

   /**
    * Compute a capture region that is shrunken with the safety heuristics.
    * @param stanceSide current stance side
    * @param capturePoint current capture point location
    * @param stancePosition center of the stance foot in the world
    * @param captureRegion current capture region to shrink.
    */
   public void computeCaptureRegionWithSafetyHeuristics(RobotSide stanceSide,
                                                        FramePoint2DReadOnly capturePoint,
                                                        FramePoint2DReadOnly stancePosition,
                                                        FrameConvexPolygon2DReadOnly captureRegion)
   {
      saferCaptureRegion.setIncludingFrame(captureRegion);

      lineOfMinimalAction.setIncludingFrame(stancePosition, capturePoint);
      lineOfMinimalAction.changeFrame(captureRegion.getReferenceFrame());
      yoLineOfMinimalAction.setMatchingFrame(lineOfMinimalAction);

      this.stancePosition.setIncludingFrame(stancePosition);
      this.stancePosition.changeFrame(captureRegion.getReferenceFrame());

      projectVerticesTowardsTheMiddle(stanceSide, captureRegion);

      if (computeVisibiltyOfVerticesFromStance(saferCaptureRegion))
         projectVerticesVisibleToStanceAwayFromTheFoot();

      saferCaptureRegion.update();

      yoSafetyBiasedCaptureRegion.setMatchingFrame(saferCaptureRegion, false);
   }

   /**
    * This method computes which vertices are visible to the stance foot. These are the vertices that will then be pushed into the
    * capture region by a distance of {@link #extraDistanceToStepFromStanceFoot}.
    */
   private boolean computeVisibiltyOfVerticesFromStance(FrameConvexPolygon2DBasics oneStepCaptureRegion)
   {
      verticesVisibleFromStance.clear();

      oneStepCaptureRegion.checkReferenceFrameMatch(stancePosition);
      int lineOfSightStartIndex = oneStepCaptureRegion.lineOfSightStartIndex(stancePosition);
      int lineOfSightEndIndex = oneStepCaptureRegion.lineOfSightEndIndex(stancePosition);
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return false;

      int index = lineOfSightEndIndex;

      while (true)
      {
         verticesVisibleFromStance.add(oneStepCaptureRegion.getVertexUnsafe(index));
         index = oneStepCaptureRegion.getPreviousVertexIndex(index);
         if (index == lineOfSightStartIndex)
         {
            verticesVisibleFromStance.add(oneStepCaptureRegion.getVertexUnsafe(index));
            break;
         }
      }

      return true;
   }

   /**
    * This method projects the visible vertices into the  capture region by a distance of {@link #extraDistanceToStepFromStanceFoot}.
    */
   private void projectVerticesVisibleToStanceAwayFromTheFoot()
   {
      // if it's zero, don't bother projecting
      if (extraDistanceToStepFromStanceFoot.getValue() <= 0.0)
         return;

      vectorToVertex.setReferenceFrame(stancePosition.getReferenceFrame());

      for (int i = 0; i < verticesVisibleFromStance.size(); i++)
      {
         FixedFramePoint2DBasics vertexToProject = verticesVisibleFromStance.get(i);
         vectorToVertex.sub(vertexToProject, stancePosition);

         // if you're already near the reachability limit, don't do any projection, it's just likely to mess you up.
         if (vectorToVertex.norm() > reachabilityLimit.getValue() - 0.42 * extraDistanceToStepFromStanceFoot.getValue())
            continue;

         // Compute the maximum distance that the vertex can be projected along the line before hitting the reachability limit.
         double maxProjectionDistance = Math.max(findMaximumProjectionDistance(vectorToVertex.norm(),
                                                                               reachabilityLimit.getValue(),
                                                                               vectorToVertex.angle(lineOfMinimalAction.getDirection())), 0.0);

         // project all the vertices visible to the foot away from the stance foot in the direction through the current ICP.
         vertexToProject.scaleAdd(Math.min(maxProjectionDistance, extraDistanceToStepFromStanceFoot.getValue()),
                                  lineOfMinimalAction.getDirection(),
                                  vertexToProject);
         saferCaptureRegion.notifyVerticesChanged();
      }

      saferCaptureRegion.update();
      boolean notifyOfUpdate = false;

      boolean visibleIndicesAreClockWise = checkIfClockWiseOrdered(verticesVisibleFromStance, saferCaptureRegion);
      int firstVisibleIndex = visibleIndicesAreClockWise ? 0 : verticesVisibleFromStance.size() - 1;
      int lastVisibleIndex = visibleIndicesAreClockWise ? verticesVisibleFromStance.size() - 1 : 0;

      // make sure the first vertex doesn't cause a "wrinkle" by overextending the outer edge of the polygon.
      {
         int firstVertexIndex = saferCaptureRegion.getClosestVertexIndex(verticesVisibleFromStance.get(firstVisibleIndex));
         Point2DReadOnly previousVertex = saferCaptureRegion.getPreviousVertex(firstVertexIndex);
         Point2DReadOnly previousPreviousVertex = saferCaptureRegion.getPreviousVertex(saferCaptureRegion.getPreviousVertexIndex(firstVertexIndex));
         tempVector.sub(previousVertex, previousPreviousVertex);
         croppedPoint.setReferenceFrame(saferCaptureRegion.getReferenceFrame());
         if (EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(previousVertex,
                                                                       tempVector,
                                                                       saferCaptureRegion.getVertex(firstVertexIndex),
                                                                       saferCaptureRegion.getNextVertex(firstVertexIndex),
                                                                       croppedPoint))
         {
            saferCaptureRegion.getVertexUnsafe(firstVertexIndex).set(croppedPoint);
            notifyOfUpdate = true;
         }
      }

      // make sure the last vertex doesn't cause a "wrinkle" by overextending the outer edge of the polygon.
      {
         int lastVertexIndex = saferCaptureRegion.getClosestVertexIndex(verticesVisibleFromStance.get(lastVisibleIndex));
         Point2DReadOnly nextVertex = saferCaptureRegion.getNextVertex(lastVertexIndex);
         Point2DReadOnly nextNextVertex = saferCaptureRegion.getNextVertex(saferCaptureRegion.getNextVertexIndex(lastVertexIndex));
         tempVector.sub(nextVertex, nextNextVertex);
         croppedPoint.setReferenceFrame(saferCaptureRegion.getReferenceFrame());
         if (EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(nextVertex,
                                                                           tempVector,
                                                                           saferCaptureRegion.getVertex(lastVertexIndex),
                                                                           saferCaptureRegion.getPreviousVertex(lastVertexIndex),
                                                                           croppedPoint))
         {
            saferCaptureRegion.getVertexUnsafe(lastVertexIndex).set(croppedPoint);
            notifyOfUpdate = true;
         }
      }

      if (notifyOfUpdate)
         saferCaptureRegion.notifyVerticesChanged();
   }

   private static boolean checkIfClockWiseOrdered(List<? extends Point2DReadOnly> points, ConvexPolygon2DReadOnly captureRegion)
   {
      if (points.size() < 2)
         return true;
      else if (points.size() > 2)
         return EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(points.get(2), points.get(0), points.get(1));
      else
      {
         if (captureRegion.getClosestVertexIndex(points.get(1)) > captureRegion.getClosestVertexIndex(points.get(0)))
            return captureRegion.isClockwiseOrdered();
         else
            return !captureRegion.isClockwiseOrdered();
      }
   }

   /**
    * Use the triangle rules to figure out how far along the current projection angle you can go before hitting your reachability limit.
    */
   private static double findMaximumProjectionDistance(double distanceToPointToProject, double maxDistanceToProjectedPoint, double projectionAngle)
   {
      double A = maxDistanceToProjectedPoint;
      double a = Math.PI - Math.abs(projectionAngle);
      double B = distanceToPointToProject;
      double sinA = Math.sin(a);
      double b = Math.asin(B * sinA / A);
      double c = Math.PI - b - a;
      return Math.sin(c) * A / sinA;
   }

   /**
    * Shrinks the vertices of the capture region towards the "line of minimal action", which is the line that goes from the middle of the foot through
    * the capture point. Theoretically, this line requires no CoP feedback. So by shrinking towards this line, you're reducing the amount of affect
    * that you allow CoP feedback to have on computing where to step.
    */
   private void projectVerticesTowardsTheMiddle(RobotSide stanceSide, FrameConvexPolygon2DReadOnly originalCaptureRegion)
   {
      // first, figure out which side the "inside" of the line of action is. This is the side of the line that faces the stance foot.
      RobotSide insideSideOfLine;
      if (lineOfMinimalAction.getDirectionX() > 0.0)
         insideSideOfLine = stanceSide;
      else
         insideSideOfLine = stanceSide.getOppositeSide();

      // reset the vector that faces forward in the world.
      forwardVector.setToZero(lineOfMinimalAction.getReferenceFrame());
      forwardVector.setX(1.0);

      // as the step goes more sideways, we want to reduce the additional amount that the "inside" vertices are shrunk towwards zero. This is equivalent
      // checking how forward the line is.
      double alpha = Math.abs(lineOfMinimalAction.getDirection().dot(forwardVector));
      double distanceIntoCaptureRegionForInsidePoints = InterpolationTools.linearInterpolate(distanceIntoCaptureRegionForEverywhere.getValue(),
                                                                                             distanceIntoCaptureRegionForInside.getValue(),
                                                                                             alpha);

      // If you're not going to shrink the points any, then don't proceed.
      if (distanceIntoCaptureRegionForInsidePoints <= 0.0)
         return;

      projectedPoint.setReferenceFrame(lineOfMinimalAction.getReferenceFrame());

      for (int i = 0; i < saferCaptureRegion.getNumberOfVertices(); i++)
      {
         FixedFramePoint2DBasics safeVertex = saferCaptureRegion.getVertexUnsafe(i);
         lineOfMinimalAction.orthogonalProjection(safeVertex, projectedPoint);

         // check if the vertex is on the inside. If it is, use the distance for inside points. If it's not, use the distance for all the other points.
         boolean isPointOnInside = lineOfMinimalAction.isPointOnSideOfLine(safeVertex, insideSideOfLine == RobotSide.LEFT);
         double maxProjectionDistance = isPointOnInside ? distanceIntoCaptureRegionForInsidePoints : distanceIntoCaptureRegionForEverywhere.getValue();

         // If the point is closer to the line that the desired projection distance, then just orthogonally project it onto the line.
         double distanceToProjectedPoint = safeVertex.distance(projectedPoint);
         if (distanceToProjectedPoint < maxProjectionDistance)
            safeVertex.set(projectedPoint);
         else
            safeVertex.interpolate(safeVertex, projectedPoint, maxProjectionDistance / distanceToProjectedPoint);

         saferCaptureRegion.notifyVerticesChanged();

         // make sure that the project points don't end up outside the actual capture region by projecting them back in if they are.
         if (!originalCaptureRegion.isPointInside(safeVertex))
            originalCaptureRegion.orthogonalProjection(safeVertex);
      }

      saferCaptureRegion.update();
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegionWithSafetyMargin()
   {
      return yoSafetyBiasedCaptureRegion;
   }

   public FrameLine2DReadOnly getLineOfMinimalAction()
   {
      return yoLineOfMinimalAction;
   }
}
