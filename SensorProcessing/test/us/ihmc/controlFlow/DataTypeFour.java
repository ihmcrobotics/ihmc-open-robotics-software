package us.ihmc.controlFlow;

import javax.vecmath.Point3d;

public class DataTypeFour
{
   private Point3d point;
   
   public Point3d getPoint()
   {
      return point;
   }
   
   public void setPoint(Point3d point)
   {
      this.point = point;
   }
   
   public void set(DataTypeFour dataTypeFour)
   {
      this.point = dataTypeFour.point;
      
   }

   public void scale(double scaleFactor)
   {
      point.scale(scaleFactor);
   }

}

