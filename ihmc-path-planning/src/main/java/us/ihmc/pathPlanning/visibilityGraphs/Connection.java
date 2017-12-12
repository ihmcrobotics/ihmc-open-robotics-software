package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Connection
{
   private Point3D point1;
   private Point3D point2;

   public Connection(Point3DReadOnly point1, Point3DReadOnly point2)
   {
      this.point1 = new Point3D(point1);
      this.point2 = new Point3D(point2);
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