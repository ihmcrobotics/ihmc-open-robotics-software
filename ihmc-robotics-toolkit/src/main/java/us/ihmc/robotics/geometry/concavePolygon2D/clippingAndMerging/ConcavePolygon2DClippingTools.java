package us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging;

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

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

public class ConcavePolygon2DClippingTools
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

      IntersectionInfo info = new IntersectionInfo(IntersectionInfo.IntersectionType.NONE, null);

      double epsilonForOnLine = 1e-4;
      double epsilonSquared = epsilonForOnLine * epsilonForOnLine;
      for (int i = 0; i < polygonToIntersect.getNumberOfVertices(); i++)
      {
         Point2DReadOnly polygonVertex = polygonToIntersect.getVertex(i);
         Point2DReadOnly polygonNextVertex = polygonToIntersect.getNextVertex(i);

         if (EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(edgeEnd, polygonVertex, polygonNextVertex) < epsilonSquared)
         {
            info = new IntersectionInfo(IntersectionInfo.IntersectionType.END, edgeEnd);
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
            {
               candidateIntersection = polygonNextVertex;
            }

            if (candidateIntersection != null && candidateIntersection.distanceSquared(edgeStart) > epsilonSquaredForSamePoint
                && candidateIntersection.distanceSquared(edgeEnd) > epsilonSquaredForSamePoint)
            {
               return new IntersectionInfo(IntersectionType.NEW, candidateIntersection);
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

   public static class LinkedPointList
   {
      private LinkedPoint firstPoint;
      private LinkedPoint lastPoint;
      private boolean isForwardList = true;

      private final Collection<LinkedPoint> points = new HashSet<>();

      public void addPointToEnd(double x, double y)
      {
         addPointToEnd(new LinkedPoint(x, y));
      }

      public void addPointToEnd(Point2DReadOnly point)
      {
         addPointToEnd(new LinkedPoint(point));
      }

      public void addPointToEnd(LinkedPoint point)
      {
         if (points.size() == 0)
         {
            firstPoint = point;
            lastPoint = point;
            point.setPredecessor(point);
            point.setSuccessor(point);
            points.add(point);
         }
         else
         {
            insertPoint(point, lastPoint);
         }
      }

      public void clear()
      {
         points.clear();
         firstPoint = null;
         lastPoint = null;
      }

      public LinkedPoint getLinkedPointAtLocation(Point2DReadOnly location)
      {
         return points.stream().filter(linkedPoint -> linkedPoint.getPoint().epsilonEquals(location, 1e-7)).findFirst().orElse(null);
      }

      public void insertPoint(LinkedPoint pointToInsert, LinkedPoint predecessor)
      {
         LinkedPoint oldSuccessor = predecessor.getSuccessor();
         predecessor.setSuccessor(pointToInsert);
         oldSuccessor.setPredecessor(pointToInsert);
         pointToInsert.setSuccessor(oldSuccessor);
         pointToInsert.setPredecessor(predecessor);
         points.add(pointToInsert);

         if (predecessor.equals(lastPoint))
            lastPoint = pointToInsert;
      }

      public void removePoint(LinkedPoint pointToRemove)
      {
         LinkedPoint predecessor = pointToRemove.getPredecessor();
         LinkedPoint successor = pointToRemove.getSuccessor();
         predecessor.setSuccessor(successor);
         successor.setPredecessor(predecessor);

         if (pointToRemove == firstPoint)
            firstPoint = successor;
         if (pointToRemove == lastPoint)
            lastPoint = predecessor;

         points.remove(pointToRemove);
      }

      public LinkedPoint getFirstPoint()
      {
         return firstPoint;
      }

      public LinkedPoint getLastPoint()
      {
         return lastPoint;
      }

      public boolean isForwardList()
      {
         return isForwardList;
      }

      public void reverseOrder()
      {
         isForwardList = !isForwardList;
         points.forEach(LinkedPoint::reverse);
         LinkedPoint oldFirstPoint = firstPoint;
         firstPoint = lastPoint;
         lastPoint = oldFirstPoint;
      }

      public Collection<LinkedPoint> getPoints()
      {
         return points;
      }

      public Collection<LinkedPoint> getPointsCopy()
      {
         List<LinkedPoint> pointsCopy = new ArrayList<>();
         for (LinkedPoint point : points)
            pointsCopy.add(new LinkedPoint(point));
         return pointsCopy;
      }

   }

   public static class LinkedPoint
   {
      private LinkedPoint predecessor;
      private LinkedPoint successor;

      private boolean isPointAfterInsideOther = false;
      private boolean isPointBeforeInsideOther = false;

      private final Point2DBasics point = new Point2D();

      private LinkedPoint pointOnOtherList = null;

      public LinkedPoint()
      {}

      public LinkedPoint(Point2DReadOnly other)
      {
         this(other.getX(), other.getY());
      }

      public LinkedPoint(LinkedPoint other)
      {
         this(other.getPoint());

         setIsPointAfterInsideOther(other.isPointAfterInsideOther);
         setIsPointBeforeInsideOther(other.isPointBeforeInsideOther);
         setPredecessor(other.predecessor);
         setSuccessor(other.successor);
      }

      public LinkedPoint(double x, double y)
      {
         this(x, y, false, false);
      }

      public LinkedPoint(Point2DReadOnly other, boolean isPointAfterInsideOther, boolean isPointBeforeInsideOther)
      {
         this(other.getX(), other.getY(), isPointAfterInsideOther, isPointBeforeInsideOther);
      }

      public LinkedPoint(double x, double y, boolean isPointAfterInsideOther, boolean isPointBeforeInsideOther)
      {
         this.isPointAfterInsideOther = isPointAfterInsideOther;
         this.isPointBeforeInsideOther = isPointBeforeInsideOther;
         setPoint(x, y);
      }

      public void setIsPointAfterInsideOther(boolean isPointAfterInsideOther)
      {
         this.isPointAfterInsideOther = isPointAfterInsideOther;
      }

      public void setIsPointBeforeInsideOther(boolean isOutgoingIntersection)
      {
         this.isPointBeforeInsideOther = isOutgoingIntersection;
      }

      public boolean getIsIntersectionPoint()
      {
         return isPointAfterInsideOther || isPointBeforeInsideOther;
      }

      public boolean isPointAfterInsideOther()
      {
         return isPointAfterInsideOther;
      }

      public boolean isPointBeforeInsideOther()
      {
         return isPointBeforeInsideOther;
      }

      public void set(LinkedPoint other)
      {
         setPoint(other.point);
         setPredecessor(other.predecessor);
         setSuccessor(other.successor);
      }

      public void linkToOtherList(LinkedPoint pointOnOtherList)
      {
         this.pointOnOtherList = pointOnOtherList;
      }

      public boolean isLinkedToOtherList()
      {
         return pointOnOtherList != null;
      }

      public LinkedPoint getPointOnOtherList()
      {
         return pointOnOtherList;
      }

      public void reverse()
      {
         LinkedPoint oldSuccessor = successor;
         successor = predecessor;
         predecessor = oldSuccessor;

         boolean oldIsIncoming = isPointAfterInsideOther;
         isPointAfterInsideOther = isPointBeforeInsideOther;
         isPointBeforeInsideOther = oldIsIncoming;
      }

      public void setPoint(Point2DReadOnly point)
      {
         setPoint(point.getX(), point.getY());
      }

      public void setPoint(double x, double y)
      {
         this.point.set(x, y);
      }

      public void setPredecessor(LinkedPoint point)
      {
         predecessor = point;
      }

      public void setSuccessor(LinkedPoint successor)
      {
         this.successor = successor;
      }

      public LinkedPoint getPredecessor()
      {
         return predecessor;
      }

      public LinkedPoint getSuccessor()
      {
         return successor;
      }

      public Point2DReadOnly getPoint()
      {
         return point;
      }

      public boolean equals(LinkedPoint other)
      {
         if (other == this)
            return true;
         else if (other == null)
            return false;
         else
            return point.getX() == other.point.getX() && point.getY() == other.point.getY();
      }

      @Override
      public String toString()
      {
         return point.toString();
      }

   }

   static class LinkedPointListHolder
   {
      private final Collection<LinkedPoint> listAPool;
      private final Collection<LinkedPoint> listBPool;

      public LinkedPointListHolder(Collection<LinkedPoint> listAPool, Collection<LinkedPoint> listBPool)
      {
         this.listAPool = listAPool;
         this.listBPool = listBPool;
      }

      public void removePoint(LinkedPoint pointToRemove)
      {
         removePointFromList(listAPool, pointToRemove);
         removePointFromList(listBPool, pointToRemove);
      }

      private static void removePointFromList(Collection<LinkedPoint> listToEdit, LinkedPoint pointToRemove)
      {
         for (LinkedPoint other : listToEdit)
         {
            if (other.getPoint().epsilonEquals(pointToRemove.getPoint(), 1e-7))
            {
               listToEdit.remove(other);
               break;
            }
         }
      }

      public int getNumberOfPoints()
      {
         return listAPool.size() + listBPool.size();
      }
   }
}
