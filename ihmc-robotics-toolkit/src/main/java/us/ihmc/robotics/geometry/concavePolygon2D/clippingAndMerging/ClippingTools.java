package us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging;

import sun.awt.image.ImageWatched.Link;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging.IntersectionInfo.IntersectionType;

import java.util.Collection;
import java.util.HashSet;

public class ClippingTools
{
   public static LinkedPointList createLinkedPointList(ConcavePolygon2DReadOnly polygon)
   {
      LinkedPointList linkedPointList = new LinkedPointList();
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         linkedPointList.addPointToEnd(polygon.getVertex(i));
      }

      return linkedPointList;
   }

   public static ConcavePolygon2DReadOnly createConcavePolygon(LinkedPointList list)
   {
      ConcavePolygon2DBasics polygon = new ConcavePolygon2D();
      LinkedPoint point = list.getFirstPoint();
      do
      {
         polygon.addVertex(point.getPoint());
         if (list.isForwardList())
            point = point.getSuccessor();
         else
            point = point.getPredecessor();
      }
      while (!point.equals(list.getFirstPoint()));

      polygon.update();
      return polygon;
   }

   public static void linkSharedVertices(LinkedPointList listA, LinkedPointList listB, double epsilon)
   {
      double epsilonSquared = epsilon * epsilon;
      LinkedPoint linkA = listA.getFirstPoint();
      do
      {
         LinkedPoint linkB = listB.getFirstPoint();
         LinkedPoint closestLink = null;
         double smallestDistanceSquared = epsilonSquared;
         do
         {
            double distanceSquared = linkA.getPoint().distanceSquared(linkB.getPoint());
            if (distanceSquared < smallestDistanceSquared)
            {
               smallestDistanceSquared = distanceSquared;
               closestLink = linkB;
            }

            linkB = linkB.getSuccessor();
         }
         while (linkB != listB.getFirstPoint());

         if (closestLink != null)
         {
            linkA.linkToOtherList(closestLink);
            closestLink.linkToOtherList(linkA);
         }

         linkA = linkA.getSuccessor();
      }
      while (linkA != listA.getFirstPoint());
   }

   private static final double epsilonForSamePoint = 1e-6;
   private static final double epsilonSquaredForSamePoint = epsilonForSamePoint * epsilonForSamePoint;
   private static final double wiggleDistance = 1e-3;

   public static void insertIntersectionsIntoList(LinkedPointList list, ConcavePolygon2DReadOnly polygonToIntersect)
   {
      LinkedPoint startPoint = list.getFirstPoint();

      Collection<Point2DReadOnly> intersections = new HashSet<>();

      // First, check to see if the start point is an intersection
      for (int i = 0; i < polygonToIntersect.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygonToIntersect.getVertex(i);
         Point2DReadOnly nextVertex = polygonToIntersect.getNextVertex(i);

         if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(startPoint.getPoint(), vertex, nextVertex) < epsilonSquaredForSamePoint)
         {
            // So the intersection is on the edge of the other polygon. Is it an incoming or outgoing intersection?
            setIntersectionInfo(startPoint,
                                startPoint.getPredecessor().getPoint(),
                                startPoint.getPoint(),
                                startPoint.getSuccessor().getPoint(),
                                polygonToIntersect);

            intersections.add(new Point2D(startPoint.getPoint()));

            break;
         }
      }

      // now, iterate over all the other points
      while (true)
      {
         LinkedPoint nextPoint = startPoint.getSuccessor();
         IntersectionInfo intersectionInfo = findFirstIntersectionInfo(startPoint.getPoint(), nextPoint.getPoint(), polygonToIntersect, new Point2D());
         Point2DReadOnly intersection = intersectionInfo.getIntersection();

         if (intersectionInfo.getIntersectionType() == IntersectionInfo.IntersectionType.NEW)
         {
            LinkedPoint point = new LinkedPoint(intersection);
            setIntersectionInfo(point, startPoint.getPoint(), intersection, nextPoint.getPoint(), polygonToIntersect);

            intersections.add(intersection);
            list.insertPoint(point, startPoint);

            continue;
         }
         else if (intersectionInfo.getIntersectionType() == IntersectionInfo.IntersectionType.END
                  && !intersections.contains(intersectionInfo.getIntersection()))
         { // the intersection is a vertex in the current list on the other polygon. The question is, what kind of intersection is it?

            // Check to see if the corner on the list is also a corner on the other polygon
            setIntersectionInfo(nextPoint, startPoint.getPoint(), intersection, nextPoint.getSuccessor().getPoint(), polygonToIntersect);

            intersections.add(intersection);
         }

         startPoint = nextPoint;
         if (startPoint == list.getFirstPoint())
            break;
      }
   }

   private static void setIntersectionInfo(LinkedPoint pointToSet,
                                           Point2DReadOnly pointBefore,
                                           Point2DReadOnly intersection,
                                           Point2DReadOnly pointAfter,
                                           ConcavePolygon2DReadOnly polygonToIntersect)
   {
      Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(intersection, pointBefore, wiggleDistance);
      Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(intersection, pointAfter, wiggleDistance);

      boolean isBeforeOnEdge = GeometryPolygonTools.isPoint2DOnPerimeterOfSimplePolygon2D(slightlyBeforeIntersection,
                                                                                          polygonToIntersect.getVertexBufferView(),
                                                                                          polygonToIntersect.getNumberOfVertices(),
                                                                                          epsilonForSamePoint);
      boolean isAfterOnEdge = GeometryPolygonTools.isPoint2DOnPerimeterOfSimplePolygon2D(slightlyAfterIntersection,
                                                                                         polygonToIntersect.getVertexBufferView(),
                                                                                         polygonToIntersect.getNumberOfVertices(),
                                                                                         epsilonForSamePoint);

      boolean isBeforeInside = GeometryPolygonTools.isPoint2DStrictlyInsideSimplePolygon2D(slightlyBeforeIntersection,
                                                                                           polygonToIntersect.getVertexBufferView(),
                                                                                           polygonToIntersect.getNumberOfVertices(),
                                                                                           null,
                                                                                           false,
                                                                                           epsilonForSamePoint);
      boolean isAfterInside = GeometryPolygonTools.isPoint2DStrictlyInsideSimplePolygon2D(slightlyAfterIntersection,
                                                                                          polygonToIntersect.getVertexBufferView(),
                                                                                          polygonToIntersect.getNumberOfVertices(),
                                                                                          null,
                                                                                          false,
                                                                                          epsilonForSamePoint);

      boolean trulyOutsideBefore = !isBeforeOnEdge && !isBeforeInside;
      boolean trulyOutsideAfter = !isAfterOnEdge && !isAfterInside;

      if (trulyOutsideAfter)
      {
         pointToSet.setIsPointAfterInsideOther(false);
         pointToSet.setIsPointBeforeInsideOther(isBeforeInside || isBeforeOnEdge);
      }
      else if (trulyOutsideBefore)
      {
         pointToSet.setIsPointAfterInsideOther(isAfterInside || isAfterOnEdge);
         pointToSet.setIsPointBeforeInsideOther(false);
      }
      else
      {
         pointToSet.setIsPointAfterInsideOther(true);
         pointToSet.setIsPointBeforeInsideOther(true);
      }
   }

   private static IntersectionInfo findFirstIntersectionInfo(Point2DReadOnly edgeStart,
                                                             Point2DReadOnly edgeEnd,
                                                             ConcavePolygon2DReadOnly polygonToIntersect,
                                                             Point2DBasics intersectionToPack)
   {
      intersectionToPack.setToNaN();

      IntersectionInfo info = new IntersectionInfo(IntersectionInfo.IntersectionType.NONE, null, null, null);

      double epsilonForOnLine = 1e-4;
      double epsilonSquared = epsilonForOnLine * epsilonForOnLine;
      for (int i = 0; i < polygonToIntersect.getNumberOfVertices(); i++)
      {


         int next = EuclidGeometryPolygonTools.next(i, polygonToIntersect.getNumberOfVertices());

         Point2DReadOnly polygonVertex = polygonToIntersect.getVertex(i);
         Point2DReadOnly polygonNextVertex = polygonToIntersect.getNextVertex(i);

         if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(edgeEnd, polygonVertex, polygonNextVertex) < epsilonSquared)
         {
            info = new IntersectionInfo(IntersectionInfo.IntersectionType.END, edgeEnd, polygonVertex, polygonNextVertex);
         }
         else
         {
            Point2DReadOnly candidateIntersection = null;
            if (EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(edgeStart, edgeEnd, polygonVertex, polygonNextVertex, intersectionToPack))
            {
               candidateIntersection = intersectionToPack;
            }
            else if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(polygonVertex, edgeStart, edgeEnd) < epsilonSquared)
            {
               candidateIntersection = polygonVertex;
            }
            else if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(polygonNextVertex, edgeStart, edgeEnd) < epsilonSquared)
               candidateIntersection = polygonNextVertex;

            if (candidateIntersection != null && candidateIntersection.distanceSquared(edgeStart) > epsilonSquaredForSamePoint
                && candidateIntersection.distanceSquared(edgeEnd) > epsilonSquaredForSamePoint)
            {
               return new IntersectionInfo(IntersectionType.NEW, candidateIntersection, polygonVertex, polygonNextVertex);
            }
         }
      }

      return info;
   }

   private static Point2DReadOnly getPointSlightlyAfterIntersection(Point2DReadOnly intersection, Point2DReadOnly vertexAfter, double epsilon)
   {
      Vector2D direction = new Vector2D();
      direction.sub(vertexAfter, intersection);
      direction.scale(epsilon / direction.length());

      Point2D littleBefore = new Point2D();
      littleBefore.add(direction, intersection);

      return littleBefore;
   }
}
