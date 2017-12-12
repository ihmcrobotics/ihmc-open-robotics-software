package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Connection
{
   private ConnectionPoint3D point1;
   private ConnectionPoint3D point2;

   public Connection(Point3DReadOnly point1, Point3DReadOnly point2)
   {
      this.point1 = new ConnectionPoint3D(point1);
      this.point2 = new ConnectionPoint3D(point2);
   }

   public ConnectionPoint3D getSourcePoint()
   {
      return point1;
   }

   public ConnectionPoint3D getTargetPoint()
   {
      return point2;
   }
}