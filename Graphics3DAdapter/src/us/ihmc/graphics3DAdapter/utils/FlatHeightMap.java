package us.ihmc.graphics3DAdapter.utils;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class FlatHeightMap implements HeightMap
{
   private static final double LIMITS = 1e3;
   private final BoundingBox3d boundingBox = new BoundingBox3d(-LIMITS, -LIMITS, -LIMITS, LIMITS, LIMITS, 0.0);

   public double heightAt(double x, double y, double z)
   {
      return 0.0;
   }

   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
}
