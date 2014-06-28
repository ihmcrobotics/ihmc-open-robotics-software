package us.ihmc.graphics3DAdapter;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.utilities.math.geometry.BoundingBox3d;

public interface HeightMapWithNormals extends HeightMap
{
   public abstract double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack);
   public abstract BoundingBox3d getBoundingBox();
}
