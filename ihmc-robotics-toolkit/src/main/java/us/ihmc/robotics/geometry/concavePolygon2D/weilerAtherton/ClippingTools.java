package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

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

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Collectors;

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

   /*
   public static void insertIntersectionsIntoListOld(LinkedPointList list, ConcavePolygon2DReadOnly polygonToIntersect)
   {
      LinkedPoint startPoint = list.getFirstPoint();

      Point2DBasics intersection = new Point2D();
      while (true)
      {
         LinkedPoint nextPoint = startPoint.getSuccessor();
         IntersectionType intersectionKey = findFirstIntersection(startPoint.getPoint(), nextPoint.getPoint(), polygonToIntersect, intersection);

         if (intersectionKey == IntersectionType.NEW)
         {
            Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(intersection, startPoint.getPoint(), 1e-3);
            Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(intersection, nextPoint.getPoint(), 5e-4);
            IntersectionBehavior intersectionBehavior = findIntersectionBehavior(slightlyBeforeIntersection, slightlyAfterIntersection, polygonToIntersect);
            boolean incoming = intersectionBehavior == IntersectionBehavior.INCOMING || intersectionBehavior == IntersectionBehavior.BOTH;
            boolean outgoing = intersectionBehavior == IntersectionBehavior.OUTGOING || intersectionBehavior == IntersectionBehavior.BOTH;

            list.insertPoint(new LinkedPoint(intersection, incoming, outgoing), startPoint);
         }
         else
         {
            if (intersectionKey == IntersectionType.BOTH || intersectionKey == IntersectionType.START)
            {
               Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(startPoint.getPoint(),
                                                                                              startPoint.getPredecessor().getPoint(),
                                                                                              5e-4);
               Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(startPoint.getPoint(), startPoint.getSuccessor().getPoint(), 5e-4);

               IntersectionBehavior intersectionBehavior = findIntersectionBehavior(slightlyBeforeIntersection, slightlyAfterIntersection, polygonToIntersect);
               boolean incoming = intersectionBehavior == IntersectionBehavior.INCOMING || intersectionBehavior == IntersectionBehavior.BOTH;
               boolean outgoing = intersectionBehavior == IntersectionBehavior.OUTGOING || intersectionBehavior == IntersectionBehavior.BOTH;

               startPoint.setIsIncomingIntersection(incoming);
               startPoint.setIsOutgoingIntersection(outgoing);
            }
            if (intersectionKey == IntersectionType.BOTH || intersectionKey == IntersectionType.END)
            {
               Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(nextPoint.getPoint(),
                                                                                              nextPoint.getPredecessor().getPoint(),
                                                                                              5e-4);
               Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(nextPoint.getPoint(), nextPoint.getSuccessor().getPoint(), 5e-4);

               IntersectionBehavior intersectionBehavior = findIntersectionBehavior(slightlyBeforeIntersection, slightlyAfterIntersection, polygonToIntersect);
               boolean incoming = intersectionBehavior == IntersectionBehavior.INCOMING || intersectionBehavior == IntersectionBehavior.BOTH;
               boolean outgoing = intersectionBehavior == IntersectionBehavior.OUTGOING || intersectionBehavior == IntersectionBehavior.BOTH;

               nextPoint.setIsIncomingIntersection(incoming);
               nextPoint.setIsOutgoingIntersection(outgoing);
            }

            startPoint = nextPoint;
            if (startPoint == list.getFirstPoint())
               break;
         }
      }
   }

    */

   private static final double epsilonForSamePoint = 1e-7;
   private static final double epsilonSquaredForSamePoint = epsilonForSamePoint * epsilonForSamePoint;
   private static final double wiggleDistance = 5e-4;

   public static void insertIntersectionsIntoList(LinkedPointList list, ConcavePolygon2DReadOnly polygonToIntersect)
   {
      LinkedPoint startPoint = list.getFirstPoint();

      List<Point2DReadOnly> pointsList = new ArrayList<>();
      do
      {
         pointsList.add(new Point2D(startPoint.getPoint()));
         startPoint = startPoint.getSuccessor();
      }
      while (startPoint != list.getFirstPoint());

      Collection<Point2DReadOnly> intersections = new HashSet<>();

      // First, check to see if the start point is an intersection
      for (int i = 0; i < polygonToIntersect.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygonToIntersect.getVertex(i);
         Point2DReadOnly nextVertex = polygonToIntersect.getNextVertex(i);

         if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(startPoint.getPoint(), vertex, nextVertex) < epsilonSquaredForSamePoint)
         {
            // check to see if it's a corner
            double distanceToStartEdge = startPoint.getPoint().distanceSquared(vertex);
            double distanceToEndEdge = startPoint.getPoint().distanceSquared(nextVertex);
            boolean isCornerOnOtherPolygon = distanceToStartEdge < epsilonForSamePoint || distanceToEndEdge < epsilonForSamePoint;

            if (isCornerOnOtherPolygon)
            {
               startPoint.setIsIncomingIntersection(true);
               startPoint.setIsOutgoingIntersection(true);
               intersections.add(new Point2D(startPoint.getPoint()));

               break;
            }

            // So the intersection is on the edge of the other polygon. Is it an incoming or outgoing intersection?
            Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(startPoint.getPoint(), startPoint.getPredecessor().getPoint(), wiggleDistance);
            Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(startPoint.getPoint(), startPoint.getSuccessor().getPoint(), wiggleDistance);

            boolean isBeforeInside = polygonToIntersect.isPointInsideEpsilon(slightlyBeforeIntersection, epsilonForSamePoint);
            boolean isAfterInside = polygonToIntersect.isPointInsideEpsilon(slightlyAfterIntersection, epsilonForSamePoint);

            if (isAfterInside == isBeforeInside)
            {

               Point2DReadOnly slightlyBeforeIntersectionOtherEdge = getPointSlightlyAfterIntersection(startPoint.getPoint(),
                                                                                                       vertex,
                                                                                                       wiggleDistance);
               Point2DReadOnly slightlyAfterIntersectionOtherEdge = getPointSlightlyAfterIntersection(startPoint.getPoint(),
                                                                                                      nextVertex,
                                                                                                      wiggleDistance);

               boolean isOtherEdgeBeforeInside = GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(slightlyBeforeIntersectionOtherEdge,
                                                                                                     pointsList,
                                                                                                     pointsList.size(),
                                                                                                     epsilonForSamePoint);
               boolean isOtherEdgeAfterInside = GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(slightlyAfterIntersectionOtherEdge,
                                                                                                    pointsList,
                                                                                                    pointsList.size(),
                                                                                                    epsilonForSamePoint);

               startPoint.setIsIncomingIntersection(!isOtherEdgeBeforeInside);
               startPoint.setIsOutgoingIntersection(!isOtherEdgeAfterInside);
            }
            else
            {
               startPoint.setIsOutgoingIntersection(isBeforeInside);
               startPoint.setIsIncomingIntersection(isAfterInside);
            }
            intersections.add(new Point2D(startPoint.getPoint()));


            break;
         }
      }

      // now, iterate over all the other points
      while (true)
      {
         LinkedPoint nextPoint = startPoint.getSuccessor();
         IntersectionInfo intersectionInfo = findFirstIntersectionInfo(startPoint.getPoint(), nextPoint.getPoint(), polygonToIntersect, new Point2D());

         if (intersectionInfo.getIntersectionType() == IntersectionType.NEW)
         {
            Point2DReadOnly intersection = intersectionInfo.getIntersection();
            Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(intersection, startPoint.getPoint(), wiggleDistance);
            Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(intersection, nextPoint.getPoint(), wiggleDistance);

            boolean beforeInside = polygonToIntersect.isPointInsideEpsilon(slightlyBeforeIntersection, epsilonForSamePoint);
            boolean afterInside = polygonToIntersect.isPointInsideEpsilon(slightlyAfterIntersection, epsilonForSamePoint);

            intersections.add(intersection);
            list.insertPoint(new LinkedPoint(intersection, afterInside, beforeInside), startPoint);

            continue;
         }
         else if (intersectionInfo.getIntersectionType() == IntersectionType.END && !intersections.contains(intersectionInfo.getIntersection()))
         {
            // the intersection is a vertex in the current list on the other polygon. The question is, what kind of intersection is it?

            // Check to see if the corner on the list is also a corner on the other polygon
            Point2DReadOnly intersection = intersectionInfo.getIntersection();
            double distanceToStartEdge = intersection.distanceSquared(intersectionInfo.getStartVertexOfIntersectingEdge());
            double distanceToEndEdge = intersection.distanceSquared(intersectionInfo.getEndVertexOfIntersectingEdge());
            boolean isCornerOnOtherPolygon = distanceToStartEdge < epsilonForSamePoint || distanceToEndEdge < epsilonForSamePoint;

            if (isCornerOnOtherPolygon)
            {
               nextPoint.setIsIncomingIntersection(true);
               nextPoint.setIsOutgoingIntersection(true);
               intersections.add(intersection);

               startPoint = nextPoint;
               if (startPoint == list.getFirstPoint())
                  break;

               continue;
            }

            // So the intersection is on the edge of the other polygon. Is it an incoming or outgoing intersection?
            Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(intersection, startPoint.getPoint(), wiggleDistance);
            Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(intersection, nextPoint.getSuccessor().getPoint(), wiggleDistance);

            boolean isBeforeInside = polygonToIntersect.isPointInsideEpsilon(slightlyBeforeIntersection, epsilonForSamePoint);
            boolean isAfterInside = polygonToIntersect.isPointInsideEpsilon(slightlyAfterIntersection, epsilonForSamePoint);

            if (isAfterInside == isBeforeInside)
            {

               Point2DReadOnly slightlyBeforeIntersectionOtherEdge = getPointSlightlyAfterIntersection(intersection,
                                                                                                       intersectionInfo.getStartVertexOfIntersectingEdge(),
                                                                                                       wiggleDistance);
               Point2DReadOnly slightlyAfterIntersectionOtherEdge = getPointSlightlyAfterIntersection(intersection,
                                                                                                      intersectionInfo.getEndVertexOfIntersectingEdge(),
                                                                                                      wiggleDistance);

               boolean isOtherEdgeBeforeInside = GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(slightlyBeforeIntersectionOtherEdge,
                                                                                                     pointsList,
                                                                                                     pointsList.size(),
                                                                                                     epsilonForSamePoint);
               boolean isOtherEdgeAfterInside = GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(slightlyAfterIntersectionOtherEdge,
                                                                                                    pointsList,
                                                                                                    pointsList.size(),
                                                                                                    epsilonForSamePoint);

               nextPoint.setIsIncomingIntersection(!isOtherEdgeBeforeInside);
               nextPoint.setIsOutgoingIntersection(!isOtherEdgeAfterInside);
            }
            else
            {
               nextPoint.setIsOutgoingIntersection(isBeforeInside);
               nextPoint.setIsIncomingIntersection(isAfterInside);
            }
            intersections.add(intersection);
         }

         startPoint = nextPoint;
         if (startPoint == list.getFirstPoint())
            break;
      }
   }

   /*
   private static IntersectionType findFirstIntersection(Point2DReadOnly edgeStart,
                                                         Point2DReadOnly edgeEnd,
                                                         ConcavePolygon2DReadOnly polygonToIntersect,
                                                         Point2DBasics intersectionToPack)
   {
      intersectionToPack.setToNaN();

      IntersectionType type = IntersectionType.NONE;

      for (int i = 0; i < polygonToIntersect.getNumberOfVertices(); i++)
      {
         int next = EuclidGeometryPolygonTools.next(i, polygonToIntersect.getNumberOfVertices());
         if (EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(edgeStart,
                                                                      edgeEnd,
                                                                      polygonToIntersect.getVertex(i),
                                                                      polygonToIntersect.getVertex(next),
                                                                      intersectionToPack))
         {
            if (intersectionToPack.distanceSquared(edgeEnd) < 1e-7)
            {
               if (type == IntersectionType.NONE)
                  type = IntersectionType.END;
               if (type == IntersectionType.START)
                  type = IntersectionType.BOTH;
            }
            else if (intersectionToPack.distanceSquared(edgeStart) < 1e-7)
            {
               if (type == IntersectionType.NONE)
                  type = IntersectionType.START;
               if (type == IntersectionType.END)
                  type = IntersectionType.BOTH;
            }
            else
            {
               return IntersectionType.NEW;
            }
         }
      }

      return type;
   }

    */

   private static IntersectionInfo findFirstIntersectionInfo(Point2DReadOnly edgeStart,
                                                             Point2DReadOnly edgeEnd,
                                                             ConcavePolygon2DReadOnly polygonToIntersect,
                                                             Point2DBasics intersectionToPack)
   {
      intersectionToPack.setToNaN();

      IntersectionInfo info = new IntersectionInfo(IntersectionType.NONE, null, null, null);

      for (int i = 0; i < polygonToIntersect.getNumberOfVertices(); i++)
      {
         int next = EuclidGeometryPolygonTools.next(i, polygonToIntersect.getNumberOfVertices());

         if (EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(edgeStart,
                                                                      edgeEnd,
                                                                      polygonToIntersect.getVertex(i),
                                                                      polygonToIntersect.getVertex(next),
                                                                      intersectionToPack))
         {
            if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(edgeEnd, polygonToIntersect.getVertex(i), polygonToIntersect.getVertex(next)) < epsilonSquaredForSamePoint)
               info = new IntersectionInfo(IntersectionType.END, edgeEnd, polygonToIntersect.getVertex(i), polygonToIntersect.getVertex(next));
            else if (intersectionToPack.distanceSquared(edgeStart) > epsilonSquaredForSamePoint)
               return new IntersectionInfo(IntersectionType.NEW, intersectionToPack, polygonToIntersect.getVertex(i), polygonToIntersect.getVertex(next));
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

   private static IntersectionBehavior findIntersectionBehavior(Point2DReadOnly littleBefore,
                                                                Point2DReadOnly littleAfter,
                                                                ConcavePolygon2DReadOnly polygonToIntersect)
   {
      boolean beforeInside = polygonToIntersect.isPointInsideEpsilon(littleBefore, epsilonForSamePoint);
      boolean afterInside = polygonToIntersect.isPointInsideEpsilon(littleAfter, epsilonForSamePoint);

      if (beforeInside && !afterInside)
         return IntersectionBehavior.OUTGOING;
      if (!beforeInside && afterInside)
         return IntersectionBehavior.INCOMING;
      return IntersectionBehavior.BOTH;
   }

   private enum IntersectionBehavior
   {INCOMING, OUTGOING, BOTH}
}
