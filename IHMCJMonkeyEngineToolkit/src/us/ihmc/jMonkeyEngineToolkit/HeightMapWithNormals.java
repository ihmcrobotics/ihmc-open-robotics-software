package us.ihmc.jMonkeyEngineToolkit;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.HeightMap;

public interface HeightMapWithNormals extends HeightMap
{
   public abstract double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack);
}
