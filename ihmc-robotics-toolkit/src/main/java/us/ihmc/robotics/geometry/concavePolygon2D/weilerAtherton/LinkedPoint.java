package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class LinkedPoint
{
   private LinkedPoint predecessor;
   private LinkedPoint successor;

   private boolean isIntersectionPoint = false;

   private final Point2DBasics point = new Point2D();

   public LinkedPoint()
   {}

   public LinkedPoint(Point2DReadOnly other)
   {
      this(other.getX(), other.getY());
   }

   public LinkedPoint(LinkedPoint other)
   {
      this(other.getPoint());

      setIsIntersectionPoint(other.isIntersectionPoint);
      setPredecessor(other.predecessor);
      setSuccessor(other.successor);
   }

   public LinkedPoint(double x, double y)
   {
      this(x, y, false);
   }

   public LinkedPoint(Point2DReadOnly other, boolean isIntersectionPoint)
   {
      this(other.getX(), other.getY(), isIntersectionPoint);
   }

   public LinkedPoint(double x, double y, boolean isIntersectionPoint)
   {
      this.isIntersectionPoint = isIntersectionPoint;
      setPoint(x, y);
   }

   public void setIsIntersectionPoint(boolean isIntersectionPoint)
   {
      this.isIntersectionPoint = isIntersectionPoint;
   }

   public boolean getIsIntersectionPoint()
   {
      return isIntersectionPoint;
   }

   public void set(LinkedPoint other)
   {
      setPoint(other.point);
      setPredecessor(other.predecessor);
      setSuccessor(other.successor);
   }

   public void reverse()
   {
      LinkedPoint oldSuccessor = successor;
      successor = predecessor;
      predecessor = oldSuccessor;
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
