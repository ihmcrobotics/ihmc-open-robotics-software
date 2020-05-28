package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class LinkedPoint
{
   private LinkedPoint predecessor;
   private LinkedPoint successor;

   private boolean isIncomingIntersection = false;
   private boolean isOutgoingIntersection = false;

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

      setIsIncomingIntersection(other.isIncomingIntersection);
      setIsOutgoingIntersection(other.isOutgoingIntersection);
      setPredecessor(other.predecessor);
      setSuccessor(other.successor);
   }

   public LinkedPoint(double x, double y)
   {
      this(x, y, false, false);
   }

   public LinkedPoint(Point2DReadOnly other, boolean isIncomingIntersection, boolean isOutgoingIntersection)
   {
      this(other.getX(), other.getY(), isIncomingIntersection, isOutgoingIntersection);
   }

   public LinkedPoint(double x, double y, boolean isIncomingIntersection, boolean isOutgoingIntersection)
   {
      this.isIncomingIntersection = isIncomingIntersection;
      this.isOutgoingIntersection = isOutgoingIntersection;
      setPoint(x, y);
   }

   public void setIsIncomingIntersection(boolean isIncomingIntersection)
   {
      this.isIncomingIntersection = isIncomingIntersection;
   }

   public void setIsOutgoingIntersection(boolean isOutgoingIntersection)
   {
      this.isOutgoingIntersection = isOutgoingIntersection;
   }

   public boolean getIsIntersectionPoint()
   {
      return isIncomingIntersection != isOutgoingIntersection;
   }

   public boolean isIncomingIntersection()
   {
      return isIncomingIntersection;
   }

   public boolean isOutgoingIntersection()
   {
      return isOutgoingIntersection;
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

      boolean oldIsIncoming = isIncomingIntersection;
      isIncomingIntersection = isOutgoingIntersection;
      isOutgoingIntersection = oldIsIncoming;
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
