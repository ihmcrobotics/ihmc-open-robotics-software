package us.ihmc.jMonkeyEngineToolkit;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.HeightMap;

public interface HeightMapWithNormals extends HeightMap
{
   public abstract double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack);
}
