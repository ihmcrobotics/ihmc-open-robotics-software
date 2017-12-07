package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.tuple3D.Point3D;

public class Connection
{
   private Point3D point1;
   private Point3D point2;

   public Connection(Point3D point1, Point3D point2)
   {
      this.point1 = point1;
      this.point2 = point2;
   }

   public Point3D getSourcePoint()
   {
      return point1;
   }

   public Point3D getTargetPoint()
   {
      return point2;
   }
}