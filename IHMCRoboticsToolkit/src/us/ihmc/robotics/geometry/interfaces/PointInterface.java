package us.ihmc.robotics.geometry.interfaces;

import javax.vecmath.Point3d;

public interface PointInterface
{
   public abstract void getPoint(Point3d pointToPack);

   public abstract void setPoint(PointInterface point);

   public abstract void setPoint(Point3d point);
}
