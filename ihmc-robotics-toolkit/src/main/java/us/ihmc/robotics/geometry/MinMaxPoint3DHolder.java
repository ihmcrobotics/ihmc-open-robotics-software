package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class MinMaxPoint3DHolder
{
   private final Point3D minPoint = new Point3D();
   private final Point3D maxPoint = new Point3D();

   public void setMinPoint(Point3DReadOnly minPoint)
   {
      this.minPoint.set(minPoint);
   }

   public Point3DBasics getMinPoint()
   {
      return minPoint;
   }

   public void setMaxPoint(Point3DReadOnly minPoint)
   {
      this.maxPoint.set(minPoint);
   }

   public Point3DBasics getMaxPoint()
   {
      return maxPoint;
   }
}