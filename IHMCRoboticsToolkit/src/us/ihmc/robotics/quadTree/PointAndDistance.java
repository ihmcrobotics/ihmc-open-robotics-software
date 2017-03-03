package us.ihmc.robotics.quadTree;

import us.ihmc.euclid.tuple3D.Point3D;

public class PointAndDistance
{
   private final Point3D point;
   private double distance;

   public PointAndDistance(Point3D point, double distance)
   {
      this.point = point;
      this.distance = distance;
   }
   
   public double getDistance()
   {
      return distance;
   }
   
   public Point3D getPoint()
   {
      return point;
   }
   
   public void setPoint(Point3D point)
   {
      this.point.set(point);
   }

   public void setPointZ(double z)
   {
      this.point.setZ(z);
   }

   public void setDistance(double distance)
   {
      this.distance = distance;
   }
}
