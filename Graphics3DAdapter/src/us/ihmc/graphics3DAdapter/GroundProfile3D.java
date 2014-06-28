package us.ihmc.graphics3DAdapter;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.BoundingBox3d;

public interface GroundProfile3D
{
   public abstract BoundingBox3d getBoundingBox();
   public abstract boolean isClose(double x, double y, double z);
   public abstract boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack);
   public abstract HeightMap getHeightMapIfAvailable();
}
