package us.ihmc.controlFlow;

import us.ihmc.euclid.tuple3D.Point3D;

public class DataTypeFour
{
   private Point3D point;
   
   public Point3D getPoint()
   {
      return point;
   }
   
   public void setPoint(Point3D point)
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

