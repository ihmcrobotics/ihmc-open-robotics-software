package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;

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

   public static void insertIntersectionsIntoList(LinkedPointList list, ConcavePolygon2DReadOnly polygonToIntersect)
   {
      LinkedPoint startPoint = list.getFirstPoint();

      while (true)
      {
         LinkedPoint nextPoint = startPoint.getSuccessor();
         Point2DReadOnly intersection = findFirstIntersection(startPoint.getPoint(), nextPoint.getPoint(), polygonToIntersect);
         if (intersection == null)
         {
            startPoint = startPoint.getSuccessor();
            if (startPoint == list.getFirstPoint())
               break;
         }
         else
         {
            list.insertPoint(new LinkedPoint(intersection, true), startPoint);
         }
      }
   }

   private static Point2DReadOnly findFirstIntersection(Point2DReadOnly edgeStart, Point2DReadOnly edgeEnd, ConcavePolygon2DReadOnly polygonToIntersect)
   {
      Point2D intersection = new Point2D();
      for (int i = 0; i < polygonToIntersect.getNumberOfVertices(); i++)
      {
         int next = EuclidGeometryPolygonTools.next(i, polygonToIntersect.getNumberOfVertices());
         if (intersectionBetweenLineSegmentsExclusive(edgeStart,
                                                      edgeEnd,
                                                      polygonToIntersect.getVertex(i),
                                                      polygonToIntersect.getVertex(next),
                                                      intersection,
                                                      1e-7))
            return intersection;
      }

      return null;
   }

   public static void insertIntersections(LinkedPointList listA, LinkedPointList listB)
   {
      LinkedPoint startSegmentA = listA.getFirstPoint();
      Point2DBasics intersection = new Point2D();

      while (true)
      {
         LinkedPoint endSegmentA = startSegmentA.getSuccessor();

         if (insertFirstEdgeIntersection(startSegmentA.getPoint(), endSegmentA.getPoint(), listB, intersection))
         {
            listA.insertPoint(new LinkedPoint(intersection, true), startSegmentA);
         }
         else
         {
            startSegmentA = startSegmentA.getSuccessor();
            if (startSegmentA == listA.getFirstPoint())
               break;
         }
      }
   }

   public static boolean insertFirstEdgeIntersection(Point2DReadOnly edgeStart, Point2DReadOnly edgeEnd, LinkedPointList list, Point2DBasics intersectionToPack)
   {
      LinkedPoint startSegment = list.getFirstPoint();
      while (true)
      {
         LinkedPoint endSegment = startSegment.getSuccessor();
         if (intersectionBetweenLineSegmentsExclusive(edgeStart, edgeEnd, startSegment.getPoint(), endSegment.getPoint(), intersectionToPack, 1e-7))
         {
            list.insertPoint(new LinkedPoint(intersectionToPack, true), startSegment);
            return true;
         }
         else
         {
            startSegment = startSegment.getSuccessor();
            if (startSegment == list.getFirstPoint())
               return false;
         }
      }
   }

   // FIXME there's probably a much faster way to do this.
   private static boolean intersectionBetweenLineSegmentsExclusive(Point2DReadOnly edge1Start,
                                                                   Point2DReadOnly edge1End,
                                                                   Point2DReadOnly edge2Start,
                                                                   Point2DReadOnly edge2End,
                                                                   Point2DBasics intersectionToPack,
                                                                   double endPointEpsilon)
   {
      if (EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(edge1Start, edge1End, edge2Start, edge2End, intersectionToPack))
      {
//         if (intersectionToPack.distanceSquared(edge1Start) < endPointEpsilon)
//            return false;
//         if (intersectionToPack.distanceSquared(edge1End) < endPointEpsilon)
//            return false;
         if (intersectionToPack.distanceSquared(edge2Start) < endPointEpsilon)
            return false;
         if (intersectionToPack.distanceSquared(edge2End) < endPointEpsilon)
            return false;

         return true;
      }

      return false;
   }
}
