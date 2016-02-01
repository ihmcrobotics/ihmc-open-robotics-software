package us.ihmc.graphics3DAdapter;

import javax.vecmath.Vector3d;

public interface HeightMapWithNormals extends HeightMap
{
   public abstract double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack);
}
