package us.ihmc.robotics.quadTree;

import javax.vecmath.Point3d;

public class PointAndDistance
{
   private final Point3d point;
   private double distance;

   public PointAndDistance(Point3d point, double distance)
   {
      this.point = point;
      this.distance = distance;
   }
   
   public double getDistance()
   {
      return distance;
   }
   
   public Point3d getPoint()
   {
      return point;
   }
   
   public void setPoint(Point3d point)
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
