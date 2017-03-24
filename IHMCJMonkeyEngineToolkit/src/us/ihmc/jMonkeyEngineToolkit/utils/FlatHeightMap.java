package us.ihmc.jMonkeyEngineToolkit.utils;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.graphicsDescription.HeightMap;

public class FlatHeightMap implements HeightMap
{
   private static final double LIMITS = 1e3;
   private final BoundingBox3D boundingBox = new BoundingBox3D(-LIMITS, -LIMITS, -LIMITS, LIMITS, LIMITS, 0.0);

   public double heightAt(double x, double y, double z)
   {
      return 0.0;
   }

   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }
}
