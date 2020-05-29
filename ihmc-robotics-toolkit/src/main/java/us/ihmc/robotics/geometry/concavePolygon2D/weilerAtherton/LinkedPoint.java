package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class LinkedPoint
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
