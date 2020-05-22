package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.*;

public class LinkedPointList
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
      throw new RuntimeException("This is untested.");
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
