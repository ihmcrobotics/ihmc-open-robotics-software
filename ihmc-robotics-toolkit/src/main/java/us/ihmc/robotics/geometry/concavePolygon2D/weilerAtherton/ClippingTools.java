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
               Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(startPoint.getPoint(), startPoint.getPredecessor().getPoint(), 5e-4);
               Point2DReadOnly slightlyAfterIntersection = getPointSlightlyAfterIntersection(startPoint.getPoint(), startPoint.getSuccessor().getPoint(), 5e-4);

               IntersectionBehavior intersectionBehavior = findIntersectionBehavior(slightlyBeforeIntersection, slightlyAfterIntersection, polygonToIntersect);
               boolean incoming = intersectionBehavior == IntersectionBehavior.INCOMING || intersectionBehavior == IntersectionBehavior.BOTH;
               boolean outgoing = intersectionBehavior == IntersectionBehavior.OUTGOING || intersectionBehavior == IntersectionBehavior.BOTH;

               startPoint.setIsIncomingIntersection(incoming);
               startPoint.setIsOutgoingIntersection(outgoing);
            }
            if (intersectionKey == IntersectionType.BOTH || intersectionKey == IntersectionType.END)
            {
               Point2DReadOnly slightlyBeforeIntersection = getPointSlightlyAfterIntersection(nextPoint.getPoint(), nextPoint.getPredecessor().getPoint(), 5e-4);
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

   public static void insertIntersectionsIntoList(LinkedPointList list, ConcavePolygon2DReadOnly polygonToIntersect)
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
            startPoint = nextPoint;
            if (startPoint == list.getFirstPoint())
               break;
         }
      }
   }

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

   private static Point2DReadOnly getPointSlightlyAfterIntersection(Point2DReadOnly intersection, Point2DReadOnly vertexAfter, double epsilon)
   {
      Vector2D direction = new Vector2D();
      direction.sub(vertexAfter, intersection);
      direction.scale(epsilon / direction.length());

      Point2D littleBefore = new Point2D();
      littleBefore.add(direction, intersection);

      return littleBefore;
   }

   private static IntersectionBehavior findIntersectionBehavior(Point2DReadOnly littleBefore, Point2DReadOnly littleAfter,
                                                                ConcavePolygon2DReadOnly polygonToIntersect)
   {
      boolean beforeInside = polygonToIntersect.isPointInside(littleBefore);
      boolean afterInside = polygonToIntersect.isPointInside(littleAfter);

      if (beforeInside && !afterInside)
         return IntersectionBehavior.OUTGOING;
      if (!beforeInside && afterInside)
         return IntersectionBehavior.INCOMING;
      return IntersectionBehavior.BOTH;
   }

   private enum IntersectionType
   {START, END, BOTH, NEW, NONE}

   private enum IntersectionBehavior
   {INCOMING, OUTGOING, BOTH}
}
