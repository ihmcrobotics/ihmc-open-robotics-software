package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.tuple3D.Point3D;

public interface PointInterface
{
   public abstract void getPoint(Point3D pointToPack);

   public abstract void setPoint(PointInterface point);

   public abstract void setPoint(Point3D point);
}
