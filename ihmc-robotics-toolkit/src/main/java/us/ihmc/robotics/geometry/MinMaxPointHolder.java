package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class MinMaxPointHolder
{
   private final Point2D minPoint = new Point2D();
   private final Point2D maxPoint = new Point2D();

   public MinMaxPointHolder()
   {

   }

   public void setMinPoint(Point2DReadOnly minPoint)
   {
      setMinPoint(minPoint.getX(), minPoint.getY());
   }

   public void setMinPoint(double x, double y)
   {
      this.minPoint.set(x, y);
   }

   public void setMaxPoint(Point2DReadOnly maxPoint)
   {
      setMaxPoint(maxPoint.getX(), maxPoint.getY());
   }

   public void setMaxPoint(double x, double y)
   {
      maxPoint.set(x, y);
   }

   public Point2DBasics getMinPoint()
   {
      return minPoint;
   }

   public Point2DBasics getMaxPoint()
   {
      return maxPoint;
   }
}