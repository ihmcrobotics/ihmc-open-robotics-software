package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
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
public class CaptureRegionSafetyHeuristics implements SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FrameLine2D lineOfMinimalAction = new FrameLine2D();
   private final FrameConvexPolygon2D saferCaptureRegion = new FrameConvexPolygon2D();

   private final YoFrameLine2D yoLineOfMinimalAction = new YoFrameLine2D("yoLineOfMinimalAction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameConvexPolygon2D yoSafetyBiasedCaptureRegion = new YoFrameConvexPolygon2D("safetyBiasedCaptureRegion",
                                                                                                 ReferenceFrame.getWorldFrame(),
                                                                                                 30,
                                                                                                 registry);

   final DoubleParameter distanceIntoCaptureRegionForInside = new DoubleParameter("distanceIntoCaptureRegionForInside", registry, 0.05);
   private final DoubleParameter distanceIntoCaptureRegionForEverywhere = new DoubleParameter("distanceIntoCaptureRegionForEverywhere", registry, 0.02);
   final DoubleParameter extraDistanceToStepFromStanceFoot = new DoubleParameter("extraDistanceToStepFromStanceFoot", registry, 0.05);

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

      moveCaptureRegionAwayFromStanceFoot();

      saferCaptureRegion.update();

      // If the capture region that has been modified (the "safer capture region") is bad (by containing no value), we can reset it to be a line, where the line
      // runs through the current capture point from the center of the foot, and is the two intersection points with the original capture region, plus some
      // extra distance from the foot. Likely, when we moved the vertices of the safer region towards this middle line, something got messed up in the
      // calculation. This shouldn't ever happen, and there are likely no edge cases where it does, but we don't it to brick the robot if it does happen, so
      // we have this little safety check.
      if (saferCaptureRegion.getNumberOfVertices() < 1)
      {
         createTheCaptureLine(captureRegion, capturePoint, saferCaptureRegion);
      }

      yoSafetyBiasedCaptureRegion.setMatchingFrame(saferCaptureRegion, false);
   }

   private void moveCaptureRegionAwayFromStanceFoot()
   {
      if (saferCaptureRegion.getNumberOfVertices() > 3)
      {
         if (computeVisibiltyOfVerticesFromStance(saferCaptureRegion))
            projectVerticesVisibleToStanceAwayFromTheFoot();
      }
      else if (saferCaptureRegion.getNumberOfVertices() == 3)
      {
         projectClosestVertexOfTheTriangle(saferCaptureRegion);
      }
      else if (saferCaptureRegion.getNumberOfVertices() == 2)
      {
         projectClosestVertexAlongLine();
      }
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


   private void projectClosestVertexOfTheTriangle(FrameConvexPolygon2DBasics oneStepCaptureRegion)
   {
      // if it's zero, don't bother projecting
      if (extraDistanceToStepFromStanceFoot.getValue() <= 0.0)
         return;

      int closestVertexIndex = oneStepCaptureRegion.getClosestVertexIndex(stancePosition);
      Point2DBasics closestVertex = oneStepCaptureRegion.getVertexUnsafe(closestVertexIndex);

      vectorToVertex.setReferenceFrame(stancePosition.getReferenceFrame());
      vectorToVertex.sub(closestVertex, stancePosition);

      // if you're already near the reachability limit, don't do any projection, it's just likely to mess you up.
      double distanceSquared = vectorToVertex.normSquared();
      if (distanceSquared > MathTools.square(reachabilityLimit.getValue()) - 0.42 * extraDistanceToStepFromStanceFoot.getValue())
         return;

      // Compute the maximum distance that the vertex can be projected along the line before hitting the reachability limit.
      double maxProjectionDistance = Math.max(findMaximumProjectionDistance(Math.sqrt(distanceSquared),
                                                                            reachabilityLimit.getValue(),
                                                                            vectorToVertex.angle(lineOfMinimalAction.getDirection())), 0.0);

      // TODO get distance along this line to the line formed by the other two vertices. That is then also the max distance.


      // project all the vertices visible to the foot away from the stance foot in the direction through the current ICP.
      closestVertex.scaleAdd(Math.min(maxProjectionDistance, extraDistanceToStepFromStanceFoot.getValue()),
                             lineOfMinimalAction.getDirection(),
                             closestVertex);
      saferCaptureRegion.notifyVerticesChanged();

      saferCaptureRegion.update();
      boolean notifyOfUpdate = false;

      return;
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

   private void projectClosestVertexAlongLine()
   {
      double totalLength = saferCaptureRegion.getVertex(0).distance(saferCaptureRegion.getVertex(1));
      double maxScaleDistance = Math.min(extraDistanceToStepFromStanceFoot.getValue(), totalLength);

      int indexToModify;
      if (saferCaptureRegion.getVertex(0).distanceSquared(stancePosition) < saferCaptureRegion.getVertex(1).distanceSquared(stancePosition))
      {
         indexToModify = 0;
      }
      else
      {
         indexToModify = 1;
      }
      tempVector.setReferenceFrame(saferCaptureRegion.getReferenceFrame());
      tempVector.sub(saferCaptureRegion.getVertex(indexToModify), stancePosition);
      tempVector.scale(maxScaleDistance / tempVector.norm());

      saferCaptureRegion.getVertexUnsafe(indexToModify).add(tempVector);
      saferCaptureRegion.notifyVerticesChanged();
   }

   private final FramePoint2D firstPoint = new FramePoint2D();
   private final FramePoint2D secondPoint = new FramePoint2D();

   private void createTheCaptureLine(FrameConvexPolygon2DReadOnly oneStepCaptureRegion,
                                     FramePoint2DReadOnly capturePoint,
                                     FrameConvexPolygon2DBasics captureLineToPack)
   {
      int intersections = oneStepCaptureRegion.intersectionWith(lineOfMinimalAction, firstPoint, secondPoint);
      captureLineToPack.clear();
      if (intersections == 0)
         captureLineToPack.addVertexMatchingFrame(capturePoint, false);
      else if (intersections == 1)
         captureLineToPack.addVertexMatchingFrame(firstPoint, false);
      else
      {
         captureLineToPack.addVertexMatchingFrame(firstPoint, false);
         captureLineToPack.addVertexMatchingFrame(secondPoint, false);
      }

      captureLineToPack.update();
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
   static double findMaximumProjectionDistance(double distanceToPointToProject, double maxDistanceToProjectedPoint, double projectionAngle)
   {
      // we're already outside the distance, so zero it out
      if (distanceToPointToProject > maxDistanceToProjectedPoint - 1e-8)
         return 0.0;

      projectionAngle = Math.abs(projectionAngle);

      if (projectionAngle < 1e-5)
      { // this is a straight line, so the calculation is really easy
         return Math.max(maxDistanceToProjectedPoint - distanceToPointToProject, 0.0);
      }

      double A = maxDistanceToProjectedPoint;
      double a = Math.PI - projectionAngle;
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
      // do nothing here if it's a line
      if (originalCaptureRegion.getNumberOfVertices() < 3)
         return;

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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Safety Biased Capture Region", yoSafetyBiasedCaptureRegion, ColorDefinitions.RoyalBlue()));
      return group;
   }
}
