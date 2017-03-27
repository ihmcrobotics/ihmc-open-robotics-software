package us.ihmc.graphicsDescription;

import us.ihmc.euclid.geometry.BoundingBox3D;

public interface HeightMap
{
   public abstract double heightAt(double x, double y, double z);
   public abstract BoundingBox3D getBoundingBox();
}
