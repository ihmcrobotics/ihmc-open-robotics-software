package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple2D.Point2D;

public class MinMaxPointHolder
{
   private final Point2D minPoint = new Point2D();
   private final Point2D maxPoint = new Point2D();
   
   public MinMaxPointHolder()
   {
      
   }
   
   public void setMinPoint(Point2D minPoint)
   {
      this.minPoint.set(minPoint);
   }
   
   public Point2D getMinPoint()
   {
      return minPoint;
   }
   
   public void setMaxPoint(Point2D minPoint)
   {
      this.maxPoint.set(minPoint);
   }
   
   public Point2D getMaxPoint()
   {
      return maxPoint;
   }
}