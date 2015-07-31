package us.ihmc.graphics3DAdapter;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.BoundingBox3d;

public interface GroundProfile3D
{
   public abstract BoundingBox3d getBoundingBox();
   
   public abstract boolean isClose(double x, double y, double z);
   
   /**
    * Returns true if inside the ground object. If inside, must pack the intersection and normal. If not inside, packing those is optional.
    */
   public abstract boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack);
   
   public abstract HeightMapWithNormals getHeightMapIfAvailable();
}
